import glob
import os
os.chdir("/home/victor/Desktop/airport_inside")
for file in glob.glob("*.png"):     
    f = open(( file.rsplit( ".", 1 )[ 0 ] ) + ".txt", "w")
    f.close()
