import os

currentFolder = os.path.dirname(os.path.abspath(__file__))
originalIdePath = os.path.join(currentFolder, "originalIDE")
expandedPath = os.path.join(originalIdePath, "expanded")
linuxPath = os.path.join(expandedPath, "linux_openocd")
macosPath = os.path.join(expandedPath, "macos_openocd")
windowsPath = os.path.join(expandedPath, "windows_openocd")

untrimmedPath = [windowsPath, linuxPath, macosPath]

# untrimmedPath=[macosPath]
# trimmedPath=[macosTrimmedPath]

for i in range(len(untrimmedPath)):
    openocdPath = untrimmedPath[i]

    basename = os.path.basename(openocdPath)

    outputName = "ide220_"+basename + ".zip"

    packCommand = f'cd "{expandedPath}" && zip -r "{outputName}" "{os.path.basename(openocdPath)}"'
    
    print(f"Packing {basename} to {outputName} ...")

    print(packCommand)
    
    packResult = os.system(packCommand)
    if packResult != 0:
        print("Packing failed!")
        exit(1)
    else:
        print(f"Packing completed. file saved to {outputName}")


    continue



