import os

win32GccUrl="https://github.com/DeqingSun/arduino_core_ch32/releases/download/initBinStorage/risc-none-embed-gcc-8.2.0.zip"
win32GccChecksum="SHA-256:43DF880A1908C873A8B6AB115568C6BEAEEC41DDADEB5202703B6AA9FDE50626"
win32GccSize=267710516
linuxGccUrl="https://github.com/DeqingSun/arduino_core_ch32/releases/download/initBinStorage/wch_gcc_8_linux-8.2.0.tar.gz"
linuxGccCheckSum="SHA-256:967FB021E5689FF79A06A71C7E54FB7FACE418D5E9B27EC64F5E56687557A282"
linuxGccSize=245443061
macosGccUrl="https://github.com/DeqingSun/arduino_core_ch32/releases/download/initBinStorage/wch_gcc_macos-8.2.0.tar.gz"
macosGccCheckSum="SHA-256:E14538AE7B621DE798C2466B338A4174CCE22C9CF12E5C1684C0995931738260"
macosGccSize=160591401
urlList = [win32GccUrl, linuxGccUrl, macosGccUrl]
checksumList = [win32GccChecksum, linuxGccCheckSum, macosGccCheckSum]
sizeList = [win32GccSize, linuxGccSize, macosGccSize]

currentFolder = os.path.dirname(os.path.abspath(__file__))
originalGccPath = os.path.join(currentFolder, "original_gcc")
trimmedGccPath = os.path.join(currentFolder, "trimmed_gcc")
if not os.path.exists(originalGccPath):
    os.mkdir(originalGccPath)
if not os.path.exists(trimmedGccPath):
    os.mkdir(trimmedGccPath)

for i in range(len(urlList)):
    url = urlList[i]
    checksum = checksumList[i]
    size = sizeList[i]
    fileBasename = os.path.basename(url)
    fileDownloadedPath = os.path.join(originalGccPath, fileBasename)
    if not os.path.exists(fileDownloadedPath):
        print(f"File {fileBasename} not found in {originalGccPath}, downloading...")
        os.system(f'curl -L "{url}" -o "{fileDownloadedPath}"')
    #check checksum
    skipChecksum = True
    if not skipChecksum:
        print("Verifying checksum...")
        checksum = checksum.split(":")[-1]
        verifyCommand = f'echo "{checksum}  {fileDownloadedPath}" | shasum -a 256 --check'
        verifyResult = os.system(verifyCommand)
        if verifyResult != 0:
            print("Checksum verification failed!")
            exit(1)

    print(f"Processing {fileBasename} ...")

    if fileBasename.endswith(".tar.gz"):
        fileBasenameNoExt = fileBasename[:-len(".tar.gz")]
    elif fileBasename.endswith(".zip"):
        fileBasenameNoExt = fileBasename[:-len(".zip")]

    expandedGccPath = os.path.join(originalGccPath, fileBasenameNoExt)
    if os.path.exists(expandedGccPath):
        #delete the folder first
        print(f"Folder {expandedGccPath} already exists, deleting...")
        os.system(f"rm -rf {expandedGccPath}")
    #unzip
    if fileBasename.endswith(".zip"):
        extractCommand = f'unzip -o "{fileDownloadedPath}" -d "{originalGccPath}"'
    elif fileBasename.endswith(".tar.gz"):
        extractCommand = f'tar -xzf "{fileDownloadedPath}" -C "{originalGccPath}"'
    extractResult = os.system(extractCommand)
    if extractResult != 0:
        print("Extraction failed!")
        exit(1)
    else:
        print("Extraction completed.")

    unnecessaryArchs = ["rv32e","rv32i","rv32imafc","rv64iac","rv64imafdc","rv32eac","rv32iac","rv32imafdc","rv64im","rv64imf","rv32em","rv32im","rv32imf","rv64imac","rv32emac","rv32imaf","rv64i","rv64imafc"]
    deletePaths = ["distro-info","include","share","libexec/gcc/riscv-none-embed/8.2.0/install-tools","lib/gcc/riscv-none-embed/8.2.0/plugin","lib/python3.7","riscv-none-embed/share",]
    for arch in unnecessaryArchs:
        deletePaths.append(f"lib/gcc/riscv-none-embed/8.2.0/{arch}")
        deletePaths.append(f"riscv-none-embed/lib/{arch}")

    for deletePath in deletePaths:
        fullDeletePath = os.path.join(expandedGccPath, deletePath)
        if os.path.exists(fullDeletePath):
            print(f"Deleting {fullDeletePath}...")
            os.system(f"rm -rf {fullDeletePath}")

    #pack it back to trimmed file
    trimmedFilePath = os.path.join(trimmedGccPath, "trimmed_" + fileBasename)
    if os.path.exists(trimmedFilePath):
        print(f"Trimmed file {trimmedFilePath} already exists, deleting...")
        os.remove(trimmedFilePath)
    
    if fileBasename.endswith(".zip"):
        #packCommand = f'cd "{expandedGccPath}" && zip -r "{trimmedFilePath}" *'
        packCommand = f'cd "{os.path.dirname(expandedGccPath)}" && zip -r "{trimmedFilePath}" "{os.path.basename(expandedGccPath)}"'
    elif fileBasename.endswith(".tar.gz"):
        #packCommand = f'cd "{expandedGccPath}" && tar -czf "{trimmedFilePath}" *'
        packCommand = f'cd "{os.path.dirname(expandedGccPath)}" && tar -czf "{trimmedFilePath}" "{os.path.basename(expandedGccPath)}"'
    packResult = os.system(packCommand)
    if packResult != 0:
        print("Packing failed!")
        exit(1)
    else:
        print(f"Packing completed. Trimmed file saved to {trimmedFilePath}")
        #check size
        actualSize = os.path.getsize(trimmedFilePath)
        print(f"Original size: {size} bytes, Trimmed size: {actualSize} bytes")
        if actualSize >= size:
            print("Warning: Trimmed size is not smaller than original size!")
        else:
            print("Trimmed size is smaller than original size.")

    continue



