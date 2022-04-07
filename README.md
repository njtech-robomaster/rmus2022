# RMUS2022 - Nanjing Tech University

## Requirements
* Docker & docker-compose installed
* nvidia-docker installed
* VSCode installed
* VSCode extensions:
  * `ms-azuretools.vscode-docker`
  * `ms-vscode-remote.remote-containers`

# Development

## Setup VSCode
1. Open VSCode
2. Run 'Remote-Containers: Open Folder in Container...' command
3. Select the repository folder

## Start the simulator
1. Start a new terminal in the repository folder
2. Run `docker-compose up sim2real_server --build`

## Build & run code in dev container
1. Start a new terminal in VSCode
2. Run `build` to build
3. Run `setup` to configure the catkin workspace
4. Run whatever you like

# Build & Run
1. Run `docker-compose up --build`
