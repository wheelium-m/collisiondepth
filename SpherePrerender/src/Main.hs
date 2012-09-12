{-# LANGUAGE BangPatterns #-}
-- |Generate images of unit spheres where each pixel encodes its
-- depth. These images are saved as PNG files for previewing, and are
-- concatenated as raw float arrays into a single file.
import Codec.Picture (Image(..), writePng)
import Control.Monad.ST (ST)
import qualified Data.Vector.Storable as V
import qualified Data.Vector.Storable.Mutable as VM
import Data.Word (Word8)
import Foreign.Storable (sizeOf)
import System.Environment (getArgs)
import System.FilePath ((</>))
import System.IO (Handle, hPutBuf, hClose, openBinaryFile, IOMode(..))
import Text.Printf (printf)

-- |Fill a row of a sphere rendering. This is a helper function for
-- 'mkSphere' that takes the vector render target, an offset into that
-- vector, a temporary value @bound*bound-y*y@ that lets us quickly
-- compute @z@ from @x@, and the row bound for this row (i.e. a function
-- of the sphere radius and the current row with respect to the full
-- image height).
fillRow :: VM.STVector s Float -> Int -> Int -> Int -> ST s ()
fillRow v !offset tmp bound = go (-bound)
  where go x | x == bound = return ()
             | otherwise = let z = sqrt (fromIntegral $ tmp - x*x)
                           in VM.write v (offset+x) z >> go (x+1)

-- |Render a sphere with the given radius in pixels.
mkSphere :: Int -> V.Vector Float
mkSphere r = V.map (/fromIntegral r) $ V.create $ 
             do v <- VM.replicate (stride * stride) 0
                let go !y
                      | y == stride = return v
                      | otherwise = let !tmp = bsq - sq (y-r)
                                        !bound = isqrt tmp
                                    in do fillRow v (y*stride+r) tmp bound
                                          go (y+1)
                go 0
  where bsq = r*r
        sq x = x * x
        stride = 2*r
        isqrt = floor . sqrt . fromIntegral

-- |Save a sphere rendering to a PNG file.
writeSpherePNG :: FilePath -> Int -> V.Vector Float -> IO ()
writeSpherePNG d r = (writePng f :: Image Word8 -> IO ()) 
                   . Image (2*r) (2*r) . V.map (floor . (*255))
  where f = d </> "Sphere"++printf "%03d" r++".png"

-- |Write a vector to a 'Handle'. This is used to dump the floating
-- point values into a single file in order of increasing sphere
-- radius.
writeVector :: Handle -> V.Vector Float -> IO ()
writeVector h v = V.unsafeWith v (flip (hPutBuf h) n)
  where n = V.length v * sizeOf (undefined::Float)
                                  
-- |Generate spheres up to, and including, the specified maximum
-- radius, and save the renderings to the given directory.
mkSpheres :: Int -> FilePath -> IO ()
mkSpheres n dst = do h <- openBinaryFile (dst </> "spheres.bin") WriteMode 
                     let go !r | r > n = return ()
                               | otherwise = let sph = mkSphere r
                                             in do writeSpherePNG dst r sph
                                                   writeVector h sph
                                                   go (r+1)
                     go 1
                     hClose h

main :: IO ()
main = getArgs >>= go
  where go [] = mkSpheres 128 "etc"
        go [n] = mkSpheres (read n) "etc"
        go [n, dst] = mkSpheres (read n) dst
        go _ = putStrLn $ "Usage: SpherePrerender"++
                          "[numSpheres [destinationDirectory]]"
