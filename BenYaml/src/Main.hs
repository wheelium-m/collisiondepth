{-# LANGUAGE OverloadedStrings #-}
import Blaze.ByteString.Builder
import Control.Applicative
import Data.ByteString (ByteString, hPut)
import qualified Data.ByteString.Char8 as B
import Data.List (intersperse, foldl', find)
import qualified Data.Map as M
import Data.Maybe (mapMaybe)
import Data.Monoid
import qualified Data.Traversable as T
import Data.Yaml.YamlLight hiding (parseYamlFile)
import Data.Yaml.Syck
import System.Environment (getArgs)
import System.IO (openFile, IOMode(..), hClose)

testFile :: FilePath
--testFile = "etc/pr2_body.yaml"
testFile = "../icra_2013_experiments/config/pr2_body_10cm.yaml"

data Group = Group { name    :: ByteString
                   , frame   :: ByteString
                   , spheres :: [ByteString] }
             deriving Show

data Sphere = Sphere { x        :: ByteString
                     , y        :: ByteString
                     , z        :: ByteString
                     , radius   :: ByteString
                     , priority :: ByteString }
              deriving Show

getSpheres :: YamlLight -> Group -> [Sphere]
getSpheres yml g = mapMaybe aux (spheres g)
  where aux k = lookupYL (YStr k) yml >>= parseSphere

parseSphere :: YamlLight -> Maybe Sphere
parseSphere yml = Sphere <$> (get "x")
                         <*> (get "y")
                         <*> (get "z")
                         <*> (get "radius")
                         <*> (get "priority")
  where get k = lookupYL (YStr k) yml >>= unStr

parseGroup :: YamlLight -> Maybe Group
parseGroup yml = Group <$> (get "name" >>= unStr)
                       <*> (get "frame" >>= unStr)
                       <*> (B.split ' ' <$> (get "spheres" >>= unStr ))
  where get k = lookupYL (YStr k) yml

dumpGroup :: YamlLight -> Group -> Builder
-- dumpGroup yml g = mconcat $ fromByteString (B.snoc (frame g) ' ') : sphereDump
--   where sphereDump = maybe [] (concatMap dumpSphere) (getSpheres yml g)
--         dumpSphere s = map fromByteString ["(", x s, " ", y s, " ", z s, " ", radius s, ") "]
dumpGroup yml g = mconcat . intersperse (fromByteString "\n") $ 
                  fromByteString (B.snoc (frame g) ' ' <> n) : sphereDump
  where sphereDump = map dumpSphere $ getSpheres yml g
        dumpSphere :: Sphere -> Builder
        dumpSphere s = mconcat $ 
                       map fromByteString ["(", x s, " ", y s, " ", z s, " ", radius s, ")"]
        n = B.pack . show . length . spheres $ g


mergeCoincidentGroups :: [Group] -> [Group]
mergeCoincidentGroups = map snd . M.toList . foldl' aux mempty . map (\g -> (frame g, g))
  where aux = flip . uncurry $ M.insertWith mergeGroups
        mergeGroups g1 g2 = g1 { spheres = spheres g1 ++ spheres g2 }

main :: IO ()
main = do fileName <- getFileName <$> getArgs
          yml <- fromYamlNode <$> parseYamlFile fileName
          let Just groups' = lookupYL (YStr "groups") yml 
                            >>= unSeq >>= mapM parseGroup
              groups = mergeCoincidentGroups groups'
          h <- openFile "etc/pr2_body.txt" WriteMode
          mapM_ (toByteStringIO (hPut h) . (<> fromByteString "\n") . dumpGroup yml)
                groups
          mapM_ (\g -> putStrLn $ B.unpack (name g) ++
                                  "("++B.unpack (frame g)++"): "++
                                  show (length (spheres g))) 
                groups
          T.traverse (print . spheres) $ find ((== "base_link") . frame) groups
          hClose h
  where getFileName [f] = f
        getFileName _ = testFile