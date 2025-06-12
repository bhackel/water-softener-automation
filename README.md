# Water Softener Automation

![IMG_3203](https://github.com/user-attachments/assets/474e1317-a1d6-4cd5-8152-395b3d486294)

This is a preliminary readme to inform about the tools used in this project. More will be added in the future.

To run, use the following command
```
make run
```

### Libraries

- **wiringPi**
  - **Usage**: Provides GPIO, PWM, and I2C functions for the Raspberry Pi.
  - **Install**: `sudo apt-get install wiringpi`

- **SQLite3**
  - **Usage**: Stores TDS and Temperature values over time
  - **Install**: `sudo apt-get install libsqlite3-dev`

- **GNU libmicrohttpd**
  - **Usage**: Hosts a web server to show the stored TDS and Temperature values.
  - **Install**: `sudo apt-get install libmicrohttpd-dev`

**Install everything**:
```
sudo apt-get update
sudo apt-get install wiringpi libsqlite3-dev libmicrohttpd-dev build-essential
```
