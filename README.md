Use `deploy_ota.sh` to build and deploy binary to the OTA server. You will need
to define the `csiwiki` SSH host to point to the CSI Wiki server in your
`~/.ssh/config`. The script will generate the file `main/include/ota_version.h`.

Before running the code, use 'git submodule' and 'git submodule update' to install all requirements.
