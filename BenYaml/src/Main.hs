{-# LANGUAGE OverloadedStrings #-}
import Blaze.ByteString.Builder
import Control.Applicative
import Data.ByteString (ByteString, hPut)
import qualified Data.ByteString.Char8 as B
import Data.List (intersperse, foldl')
import qualified Data.Map as M
import Data.Monoid
import Data.Yaml.YamlLight hiding (parseYamlFile)
import Data.Yaml.Syck
import System.IO (openFile, IOMode(..), hClose)

testFile :: FilePath
testFile = "etc/pr2_body.yaml"

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

getSpheres :: YamlLight -> Group -> Maybe [Sphere]
getSpheres yml g = mapM aux (spheres g)
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
dumpGroup yml g = mconcat $ intersperse (fromByteString " ") 
                  (fromByteString (frame g) : sphereDump)
  where sphereDump = maybe [] (concatMap dumpSphere) (getSpheres yml g)
        dumpSphere s = map fromByteString ["(", x s, y s, z s, radius s, ")"]

mergeCoincidentGroups :: [Group] -> [Group]
mergeCoincidentGroups = map snd . M.toList . foldl' aux mempty . map (\g -> (frame g, g))
  where aux = flip . uncurry $ M.insertWith mergeGroups
        mergeGroups g1 g2 = g1 { spheres = spheres g1 ++ spheres g2 }

main :: IO ()
main = do yml <- fromYamlNode <$> parseYamlFile testFile
          let Just groups' = lookupYL (YStr "groups") yml 
                            >>= unSeq >>= mapM parseGroup
              groups = mergeCoincidentGroups groups'
          h <- openFile "etc/pr2_body.txt" WriteMode
          mapM_ (toByteStringIO (hPut h) . (<> fromByteString "\n") . dumpGroup yml)
                groups
          print groups
          hClose h
