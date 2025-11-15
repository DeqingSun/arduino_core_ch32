import os

currentFolder = os.path.dirname(os.path.abspath(__file__))
originalIdePath = os.path.join(currentFolder, "originalIDE")
expandedPath = os.path.join(originalIdePath, "expanded")
linuxPath = os.path.join(expandedPath, "linux_toolchain")
macosPath = os.path.join(expandedPath, "macos_toolchain")
windowsPath = os.path.join(expandedPath, "windows_toolchain")
linuxTrimmedPath = os.path.join(expandedPath, "trimmed_linux_toolchain")
macosTrimmedPath = os.path.join(expandedPath, "trimmed_macos_toolchain")
windowsTrimmedPath = os.path.join(expandedPath, "trimmed_windows_toolchain")

untrimmedPath = [windowsPath, linuxPath, macosPath]
trimmedPath = [windowsTrimmedPath, linuxTrimmedPath, macosTrimmedPath]

untrimmedPath=[macosPath]
trimmedPath=[macosTrimmedPath]

for i in range(len(untrimmedPath)):
    originalGccPath = untrimmedPath[i]
    trimmedGccPath = trimmedPath[i]
    basename = os.path.basename(originalGccPath)
    osType = basename.split("_")[0]
    print(f"Processing {osType} toolchain in {basename}")
    if not os.path.exists(originalGccPath):
        exit(f"Original GCC path {originalGccPath} does not exist, please run extractIDE220.py first to extract the IDE toolchains.")
    if os.path.exists(trimmedGccPath):
        os.system(f"rm -rf {trimmedGccPath}")
    os.mkdir(trimmedGccPath)
    oneLevelPath = os.path.join(trimmedGccPath, "trim_gcc_ide_220")
    os.mkdir(oneLevelPath)
    #copy all files from originalGccPath to oneLevelPath
    os.system(f"cp -r {originalGccPath}/* {oneLevelPath}/")

    unnecessaryArchs = ["rv32e","rv32i","rv32imafc","rv64iac","rv64imafdc","rv32eac","rv32iac","rv32imafdc","rv64im","rv64imf","rv32em","rv32im","rv32imf","rv64imac","rv32emac","rv32imaf","rv64i","rv64imafc"]
    deletePaths = ["distro-info","include","share","libexec/gcc/riscv-none-embed/8.2.0/install-tools","lib/gcc/riscv-none-embed/8.2.0/plugin","lib/python3.7","riscv-none-embed/share",]
    for arch in unnecessaryArchs:
        deletePaths.append(f"lib/gcc/riscv-none-embed/8.2.0/{arch}")
        deletePaths.append(f"riscv-none-embed/lib/{arch}")

    for deletePath in deletePaths:
        fullDeletePath = os.path.join(oneLevelPath, deletePath)
        if os.path.exists(fullDeletePath):
            print(f"Deleting {fullDeletePath}...")
            os.system(f"rm -rf {fullDeletePath}")

    #pack it back to trimmed file
    trimmedFilePath = os.path.join(trimmedGccPath, "trimmed_" + osType + "_toolchain_ide_220.zip")
    if os.path.exists(trimmedFilePath):
        print(f"Trimmed file {trimmedFilePath} already exists, deleting...")
        os.remove(trimmedFilePath)
    

    packCommand = f'cd "{os.path.dirname(oneLevelPath)}" && zip -r "{trimmedFilePath}" "{os.path.basename(oneLevelPath)}"'
    packResult = os.system(packCommand)
    if packResult != 0:
        print("Packing failed!")
        exit(1)
    else:
        print(f"Packing completed. Trimmed file saved to {trimmedFilePath}")
        #check size
        actualSize = os.path.getsize(trimmedFilePath)
        print(f"Trimmed size: {actualSize} bytes")

    continue



