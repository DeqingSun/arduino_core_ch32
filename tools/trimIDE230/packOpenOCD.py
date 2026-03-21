import os

currentFolder = os.path.dirname(os.path.abspath(__file__))
originalIdePath = os.path.join(currentFolder, "originalIDE")
expandedPath = os.path.join(originalIdePath, "expanded")
linuxPath = os.path.join(expandedPath, "linux_openocd")
macosPath = os.path.join(expandedPath, "macos_openocd")
windowsPath = os.path.join(expandedPath, "windows_openocd")

untrimmedPath = [macosPath, windowsPath, linuxPath]

# untrimmedPath=[macosPath]
# trimmedPath=[macosTrimmedPath]

for i in range(len(untrimmedPath)):
    openocdPath = untrimmedPath[i]

    basename = os.path.basename(openocdPath)

    outputName = "ide230_"+basename + ".zip"

    excludeList = ["contrib", "doc", "distro-info", "scripts", "share"]
    excludePattern = ""
    for item in excludeList:
        excludePattern += f'"{basename}/OpenOCD/OpenOCD/{item}/*" '

    packCommand = f'cd "{expandedPath}" && zip -r "{outputName}" "{basename}" -x {excludePattern}'
    
    print(f"Packing {basename} to {outputName} ...")

    print(packCommand)
    
    packResult = os.system(packCommand)
    if packResult != 0:
        print("Packing failed!")
        exit(1)
    else:
        print(f"Packing completed. file saved to {outputName}")


    continue



