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

# untrimmedPath=[macosPath]
# trimmedPath=[macosTrimmedPath]

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

    #list all rv* folders in riscv-wch-elf/lib/
    riscvLibPath = os.path.join(oneLevelPath, "riscv-wch-elf", "lib")
    if not os.path.exists(riscvLibPath):
        print(f"riscv-none-embed lib path {riscvLibPath} does not exist, can not determine architectures to delete.")
        exit(1)
    archs = []
    for item in os.listdir(riscvLibPath):
        itemPath = os.path.join(riscvLibPath, item)
        if os.path.isdir(itemPath) and item.startswith("rv"):
            archs.append(item)
    print(f"Detected architectures: {archs}")

    #rv32ecxw ilp32e
    #rv32imacxw ilp32
    #rv32imac ilp32
    #rv32imafcxw ilp32f
    #rv32imc_zba_zbb_zbc_zbs_xw ilp32

    usefulArchs = ["rv32ec_xw","rv32imac_xw","rv32imafc_xw","rv32imc_zba_zbb_zbc_zbs_xw"]

    unnecessaryArchs = []
    for arch in archs:
        if arch not in usefulArchs:
            unnecessaryArchs.append(arch)

    deletePaths = ["distro-info","include","share","libexec/gcc/riscv-wch-elf/12.2.0/install-tools","lib/gcc/riscv-wch-elf/12.2.0/plugin","lib/python3.7","riscv-wch-elf/share",]
    for arch in unnecessaryArchs:
        deletePaths.append(f"lib/gcc/riscv-wch-elf/12.2.0/{arch}")
        deletePaths.append(f"riscv-wch-elf/lib/{arch}")
        deletePaths.append(f"riscv-wch-elf/picolibc/riscv-wch-elf/lib/{arch}")

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



