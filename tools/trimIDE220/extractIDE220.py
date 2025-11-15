import os
import subprocess

#go to https://www.mounriver.com/download to get 220 IDE.
#it seems protected, so we cannot download it automatically.

#we need to extract following accroding to https://github.com/arduino/arduino-cli/blob/master/docs/package_index_json-specification.md
# x86_64-mingw32
# arm64-apple-darwin
# x86_64-linux-gnu

windowsIdeFile = "MounRiver_Studio_Setup_V220.zip"
macosIdeFile = "MounRiver_Studio_MacOS_ARM64_V220.tar.gz"
linuxIdeFile = "MounRiverStudio_Linux_X64_V220.tar.xz"

doExtract = True

currentFolder = os.path.dirname(os.path.abspath(__file__))
originalIdePath = os.path.join(currentFolder, "originalIDE")

if not os.path.exists(originalIdePath):
    exit(f"originalIdePath {originalIdePath} does not exist.")

extractedWindowsPack = os.path.join(originalIdePath, "expanded/Toolchain.7z")
path7zz = os.path.join(originalIdePath, "7zz")
#check macOS IDE tar.gz file
macosfileDownloadedPath = os.path.join(originalIdePath, macosIdeFile)
if not os.path.exists(macosfileDownloadedPath):
    exit(f"File {macosIdeFile} not found in {originalIdePath}, please download it from https://www.mounriver.com/download and place it there.")
#check Linux IDE tar.xz file
linuxfileDownloadedPath = os.path.join(originalIdePath, linuxIdeFile)
if not os.path.exists(linuxfileDownloadedPath):
    exit(f"File {linuxIdeFile} not found in {originalIdePath}, please download it from https://www.mounriver.com/download and place it there.")

if os.path.exists(extractedWindowsPack):
    print(f"Windows Toolchain.7z already extracted to {extractedWindowsPack}, skip extraction.")
else:
    #deal with windows IDE zip file
    windowsfileDownloadedPath = os.path.join(originalIdePath, windowsIdeFile)
    if not os.path.exists(windowsfileDownloadedPath):
        exit(f"File {windowsIdeFile} not found in {originalIdePath}, please download it from https://www.mounriver.com/download and place it there.")

    #check if MounRiver_Studio_Setup_V220.exe is in the zip file
    listing = subprocess.check_output(["unzip", "-l", windowsfileDownloadedPath], text=True)
    if not "MounRiver_Studio_Setup_V220.exe" in listing:
        exit("MounRiver_Studio_Setup_V220.exe not found in the zip file, please check the zip file.")

    #extract the exe file to originalIdePath/expanded
    expandedIdePath = os.path.join(originalIdePath, "expanded")
    if not os.path.exists(expandedIdePath):
        os.mkdir(expandedIdePath)
    targetExePath = os.path.join(expandedIdePath, "MounRiver_Studio_Setup_V220.exe")
    if not os.path.exists(targetExePath):
        extractCommand = f'unzip -o "{windowsfileDownloadedPath}" "MounRiver_Studio_Setup_V220.exe" -d "{expandedIdePath}"'
        extractResult = os.system(extractCommand)
        if extractResult != 0:
            exit("Extraction of exe file failed!")

    #7zz is from https://www.7-zip.org/a/7z2501-mac.tar.xz
    listing = subprocess.check_output([path7zz, "l", targetExePath], text=True)
    if not "resources/app/resources/win32/components/WCH/Toolchain.7z" in listing:
        exit("Toolchain.7z not found in the exe file, please check the exe file.")
    #extract Toolchain.7z
    if not os.path.exists(os.path.join(expandedIdePath, "Toolchain.7z")):
        extractCommand = f'"{path7zz}" e -y -o"{expandedIdePath}" "{targetExePath}" "resources/app/resources/win32/components/WCH/Toolchain.7z"'
        extractResult = os.system(extractCommand)
        if extractResult != 0:
            exit("Extraction of Toolchain.7z failed!")


listing = subprocess.check_output(["tar", "-tf", macosfileDownloadedPath], text=True)
if not "./MounRiver Studio 2.app/Contents/Resources/app/resources/darwin/components/WCH/Toolchain/RISC-V Embedded GCC12/" in listing:
    exit("Toolchain folder not found in the tar.gz file, please check the tar.gz file.")
listing = subprocess.check_output(["tar", "-tf", linuxfileDownloadedPath], text=True)
if not "MRS-linux-x64/resources/app/resources/linux/components/WCH/Toolchain/RISC-V Embedded GCC12/" in listing:
    exit("Toolchain folder not found in the tar.xz file, please check the tar.xz file.")

#remove old extracted toolchain
extractedWindowsExe = os.path.join(originalIdePath, "expanded/MounRiver_Studio_Setup_V220.exe")
extractedWindowsPack7z = os.path.join(originalIdePath, "expanded/Toolchain.7z")
extractedWindowsPack = os.path.join(originalIdePath, "expanded/windows_toolchain")
if os.path.exists(extractedWindowsPack):
    os.system(f'rm -rf "{extractedWindowsPack}"')
extractedMacosPack = os.path.join(originalIdePath, "expanded/macos_toolchain")
#remove existing extracted folder to force re-extraction
if os.path.exists(extractedMacosPack):
    os.system(f'rm -rf "{extractedMacosPack}"')
extractedLinuxPack = os.path.join(originalIdePath, "expanded/linux_toolchain")
if os.path.exists(extractedLinuxPack):
    os.system(f'rm -rf "{extractedLinuxPack}"')

extractedWindowsOpenOCDPack = os.path.join(originalIdePath, "expanded/windows_openocd")
if os.path.exists(extractedWindowsOpenOCDPack):
    os.system(f'rm -rf "{extractedWindowsOpenOCDPack}"')
extractedMacosOpenOCDPack = os.path.join(originalIdePath, "expanded/macos_openocd")
if os.path.exists(extractedMacosOpenOCDPack):
    os.system(f'rm -rf "{extractedMacosOpenOCDPack}"')
extractedLinuxOpenOCDPack = os.path.join(originalIdePath, "expanded/linux_openocd")
if os.path.exists(extractedLinuxOpenOCDPack):
    os.system(f'rm -rf "{extractedLinuxOpenOCDPack}"')

#extract openocd
if doExtract:
    exeOpenOcdPath = "resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD/"
    #extract windows OpenOCD
    if not os.path.exists(extractedWindowsOpenOCDPack):
        os.mkdir(extractedWindowsOpenOCDPack)
    extractCommand = f'"{path7zz}" x -y -o"{extractedWindowsOpenOCDPack}" "{extractedWindowsExe}" "{exeOpenOcdPath}"'
    extractResult = os.system(extractCommand)
    if extractResult != 0:
        exit("Extraction of Windows OpenOCD failed!")
    mvCmd = f'mv "{os.path.join(extractedWindowsOpenOCDPack, exeOpenOcdPath)}"/* "{extractedWindowsOpenOCDPack}"'
    os.system(mvCmd)
    removeCmd = f'rm -rf "{os.path.join(extractedWindowsOpenOCDPack, "resources")}"'
    os.system(removeCmd)

    #extract macOS OpenOCD
    macOpenOcdPath = "./MounRiver Studio 2.app/Contents/Resources/app/resources/darwin/components/WCH/OpenOCD/OpenOCD/"
    if not os.path.exists(extractedMacosOpenOCDPack):
        os.mkdir(extractedMacosOpenOCDPack)
    extractCommand = f'tar --strip-components=11 -xzf "{macosfileDownloadedPath}" -C "{extractedMacosOpenOCDPack}" "{macOpenOcdPath}"'
    extractResult = os.system(extractCommand)
    if extractResult != 0:
        exit("Extraction of macOS OpenOCD failed!")

    #extract Linux OpenOCD
    linuxOpenOcdPath = "MRS-linux-x64/resources/app/resources/linux/components/WCH/OpenOCD/OpenOCD/"
    if not os.path.exists(extractedLinuxOpenOCDPack):
        os.mkdir(extractedLinuxOpenOCDPack)
    extractCommand = f'tar --strip-components=9 -xJf "{linuxfileDownloadedPath}" -C "{extractedLinuxOpenOCDPack}" "{linuxOpenOcdPath}"'
    extractResult = os.system(extractCommand)
    if extractResult != 0:
        exit("Extraction of Linux OpenOCD failed!")

#extract toolchains
if doExtract:
    #extract macOS Toolchain
    if not os.path.exists(extractedMacosPack):
        os.mkdir(extractedMacosPack)
    extractCommand = f'tar --strip-components=11 -xzf "{macosfileDownloadedPath}" -C "{extractedMacosPack}" "./MounRiver Studio 2.app/Contents/Resources/app/resources/darwin/components/WCH/Toolchain/RISC-V Embedded GCC12/"'
    extractResult = os.system(extractCommand)
    if extractResult != 0:
        exit("Extraction of macOS Toolchain failed!")
    #extract Linux Toolchain
    if not os.path.exists(extractedLinuxPack):
        os.mkdir(extractedLinuxPack)
    extractCommand = f'tar --strip-components=9 -xJf "{linuxfileDownloadedPath}" -C "{extractedLinuxPack}" "MRS-linux-x64/resources/app/resources/linux/components/WCH/Toolchain/RISC-V Embedded GCC12/"'
    extractResult = os.system(extractCommand)
    if extractResult != 0:
        exit("Extraction of Linux Toolchain failed!")
    #extract windows Toolchain
    if not os.path.exists(extractedWindowsPack):
        os.mkdir(extractedWindowsPack)
    extractCommand = f'"{path7zz}" x -y -o"{extractedWindowsPack}" "{extractedWindowsPack7z}" "RISC-V Embedded GCC12/"'
    extractResult = os.system(extractCommand)
    if extractResult != 0:
        exit("Extraction of Windows Toolchain failed!")
    mvCmd = f'mv "{os.path.join(extractedWindowsPack, "RISC-V Embedded GCC12")}"/* "{extractedWindowsPack}"'
    os.system(mvCmd)
    removeCmd = f'rm -rf "{os.path.join(extractedWindowsPack, "RISC-V Embedded GCC12")}"'
    os.system(removeCmd)
