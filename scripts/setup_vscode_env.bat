:: Create .vscode directory if it doesn't exist
if not exist "..\.vscode" (
    mkdir "..\.vscode"
)

:: Create or overwrite the launch.json file
> "..\.vscode\launch.json" (
    echo {
    echo     "version": "0.2.0",
    echo     "configurations": [
    echo     {
    echo         "name": "(gdb) Launch",
    echo         "type": "cppdbg",
    echo         "request": "launch",
    echo         "program": "${workspaceFolder}/build/debug/seahowl_driver",
    echo         "args": ["../data/IEA15MW/onshore/main.json"],
    echo         "stopAtEntry": false,
    echo         "cwd": "${workspaceFolder}/build",
    echo         "environment": [],
    echo         "externalConsole": false,
    echo         "MIMode": "gdb",
    echo         "setupCommands": [
    echo             {
    echo                 "description": "Enable pretty-printing for gdb",
    echo                 "text": "-enable-pretty-printing",
    echo                 "ignoreFailures": true
    echo             },
    echo             {
    echo                 "description": "Set Disassembly Flavor to Intel",
    echo                 "text": "-gdb-set disassembly-flavor intel",
    echo                 "ignoreFailures": true
    echo             }
    echo         ]
    echo     }
    echo     ]
    echo }
)

:: Create or overwrite the settings.json file
> "..\.vscode\settings.json" (
    echo {
    echo     "python.testing.unittestArgs": [
    echo         "-v",
    echo         "-s",
    echo         "tests/non_regression",
    echo         "-p",
    echo         "*.py"
    echo     ],
    echo     "python.testing.pytestEnabled": false,
    echo     "python.testing.unittestEnabled": true,
    echo     "files.associations": {
    echo         "deque": "cpp",
    echo         "string": "cpp",
    echo         "vector": "cpp"
    echo     }
    echo }
)

:: Install each extension
for %%e in (
    matepek.vscode-catch2-test-adapter
    ms-python.debugpy
    ms-python.python
    ms-python.vscode-pylance
    ms-vscode.cmake-tools
    ms-vscode.cpptools
    ms-vscode.cpptools-extension-pack
    ms-vscode.cpptools-themes
    ms-vscode.live-server
    twxs.cmake
) do (
    code --install-extension %%e
)
