#!/bin/bash

# List of extensions to install
extensions=(
"matepek.vscode-catch2-test-adapter"
"ms-python.debugpy"
"ms-python.python"
"ms-python.vscode-pylance"
"ms-vscode.cmake-tools"
"ms-vscode.cpptools"
"ms-vscode.cpptools-extension-pack"
"ms-vscode.cpptools-themes"
"ms-vscode.live-server"
"twxs.cmake"
)

# Install each extension
for extension in "${extensions[@]}"; do
    code --install-extension "$extension"
done

# create .vscode directory if it doesn't exist
mkdir -p ../.vscode
# Create or overwrite the launch.json file
cat <<EOL > ../.vscode/launch.json
{
    "version": "0.2.0",
    "configurations": [
    {
        "name": "(gdb) Launch",
        "type": "cppdbg",
        "request": "launch",
        "program": "${workspaceFolder}/build/debug/seahowl_driver",
        "args": ["../data/IEA15MW/onshore/main.json"],
        "stopAtEntry": false,
        "cwd": "${workspaceFolder}/build",
        "environment": [],
        "externalConsole": false,
        "MIMode": "gdb",
        "setupCommands": [
            {
                "description": "Enable pretty-printing for gdb",
                "text": "-enable-pretty-printing",
                "ignoreFailures": true
            },
            {
                "description": "Set Disassembly Flavor to Intel",
                "text": "-gdb-set disassembly-flavor intel",
                "ignoreFailures": true
            }
        ]
    }
    ]
}
EOL
# Create or overwrite the settings.json file
cat <<EOL > ../.vscode/settings.json
{
    "python.testing.unittestArgs": [
        "-v",
        "-s",
        "tests/non_regression",
        "-p",
        "*.py"
    ],
    "python.testing.pytestEnabled": false,
    "python.testing.unittestEnabled": true,
    "files.associations": {
        "deque": "cpp",
        "string": "cpp",
        "vector": "cpp"
    },
}
EOL
