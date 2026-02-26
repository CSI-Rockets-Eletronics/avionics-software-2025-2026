Use `deploy_ota.sh` to build and deploy binary to the OTA server. You will need
to define the `csiwiki` SSH host to point to the CSI Wiki server in your
`~/.ssh/config`. The script will generate the file `main/include/ota_version.h`.

First, follow the steps to install the ESP-IDF Extension for VS Code, found at this git repo: https://github.com/espressif/vscode-esp-idf-extension/blob/master/README.md. Select Express and choose ESP-IDF version 5.5.

Before building the project, clone the git submodules via

```
git submodule update --init --recursive
```

However, this will run into an error as the credentials to connect to the meche server are missing. To begin, add this to your ssh config (via `code ~/.ssh/config`):

```
HOST csiwiki
  HostName csiwiki.me.columbia.edu
  User wikijs
  ForwardAgent yes
```

Then, generate ssh key if needed: `ssh-keygen -t ed25519 -C "your_email@example.com"`

Now, copy it to the remote: `ssh-copy-id -i ~/.ssh/id_ed25519.pub csiwiki`

Next, idf.py must be activated. Typically, you should be able to just run `source activate`. If the envrionment remains the same, you need to run `source ~/.espressif/python_env/idf5.1_py3.9_env/bin/activate` instead. Finally, run `./deploy_ota.sh `.

## Ignoring Local VS Code Settings

The `.vscode/settings.json` file often contains computer-specific settings, such as the ESP COM port, which are automatically updated by the C++ and Espressif IDF extensions. To prevent these local changes from appearing as unstaged changes in git, you can tell git to ignore modifications to this file using:

```
git update-index --skip-worktree .vscode/settings.json
```

This will keep your local settings out of version control while still allowing the file to exist in the repository for others.

## Issues with Submodule libraries storing no content
--> when trying to create a new build for the code, the submodule component folders are showing no local content, to fix this issue:

Run:
git submodule init
git submodule update --recursive
