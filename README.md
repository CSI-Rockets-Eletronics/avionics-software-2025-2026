Use `deploy_ota.sh` to build and deploy binary to the OTA server. You will need
to define the `csiwiki` SSH host to point to the CSI Wiki server in your
`~/.ssh/config`. The script will generate the file `main/include/ota_version.h`.

First, follow the steps to install the ESP-IDF Extension for VS Code, found at this git repo: https://github.com/espressif/vscode-esp-idf-extension/blob/master/README.md. Select Express and choose ESP-IDF version 5.5. 

Before building the project, use `git submodule` and `git submodule update` to install all requirements.

However, this will run into an error as the credentials to connect to the meche server are missing. To begin, idf.py must be activated. Typically, you should be able to just run `source activate`. If the envrionment remains the same, you need to run `source ~/.espressif/python_env/idf5.1_py3.9_env/bin/activate` instead. 
