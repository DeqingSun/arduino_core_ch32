import os

#artifacts need login to download, do it manually

wchisp_win_x64_artifacts_path="wchisp-win-x64.zip"
wchisp_macos_x64_artifacts_path="wchisp-macos-x64.zip"
wchisp_linux_x64_artifacts_path="wchisp-linux-x64.zip"

script_dir = os.path.dirname(os.path.abspath(__file__))
wchisp_path = os.path.abspath(os.path.join(script_dir, "wchisp"))
os.makedirs(wchisp_path, exist_ok=True)

artifacts_path = [wchisp_win_x64_artifacts_path, wchisp_macos_x64_artifacts_path, wchisp_linux_x64_artifacts_path]
printString = "\n"
for path in artifacts_path:
    pathFull = os.path.join(wchisp_path, path)
    basename = os.path.basename(path)
    basenameNoExt = os.path.splitext(basename)[0]
    unpack_dir = os.path.join(wchisp_path, basenameNoExt)
    #delete old unpack dir
    if os.path.exists(unpack_dir):
        os.system(f"rm -rf {unpack_dir}")
    if os.path.exists(pathFull):
        os.system(f"unzip -o {pathFull} -d {unpack_dir}")
    if "win" in basename:
        #download .dll file
        dllPath = "https://github.com/DeqingSun/wchisp/raw/51738cc4aceff9dea20febe586418adc502bb8be/src/CH375DLL64.dll"
        os.system(f"curl -L {dllPath} -o {unpack_dir}/CH375DLL64.dll")

    currentDateInYYYYMMDD = os.popen("date +%Y%m%d").read().strip()
    newZipName = f"{basename.replace('.zip', '')}_{currentDateInYYYYMMDD}.zip"
    newZipPath = os.path.join(wchisp_path, newZipName)
    print(f"Creating {newZipPath} ...")
    if os.path.exists(newZipPath):
        os.remove(newZipPath)
    os.system(f"cd {unpack_dir}/.. && zip -r {newZipPath} {basenameNoExt}")

    printString += f"{newZipName} : the size and sha256 is \n"
    size = os.path.getsize(newZipPath)
    sha256 = os.popen(f"shasum -a 256 {newZipPath} | awk '{{print $1}}'").read().strip()
    printString += f"{size}\n"
    printString += f"{sha256}\n\n"
print(printString)
