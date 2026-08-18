# Horizon Groundstation
The Horizon Groundstation is an application for visualizing real-time sensor data from a rocket, including telemetry and gyro data, through a sophisticated graphical interface. Developed using C++ for data processing and QML for the front-end, this project was created to provide a reliable, real-time monitoring tool as part of the University of South-Eastern Norway’s student company, USN Horizon.

This was the first year of the project, aiming to participate in the **EuroRoc** competition in Portugal with the student rocket launch team. The purpose of this repository is to enable next year’s members of **USN Horizon** to continue developing the project, taking it to new heights.

## Table of Contents
### Groundstation
- [Introduction](#introduction)
- [Project Structure](#project-structure)
- [Features](#features)
- [Signal & Slot Mechanism](#signal--slot-mechanism)
- [Testing](#testing)
- [Future Improvements](#future-improvements)

### Setup Instructions for Developers
- [Getting Started](#getting-started)
- [Enviorment Variables](#enviorment-variables)
- [Building the Project](#building-the-project)
- [Useful Resources](#useful-resources)
- [Authors](#authors)

## Introduction

The Horizon Groundstation application processes sensor data from a rocket in real-time, converting it into graphical forms such as line charts and gyro-based animations. This allows users to monitor parameters like humidity and orientation dynamically. This documentation provides a comprehensive view of the technical details and functionality for future development and maintenance.

## Project Structure

The application is designed to separate concerns, with each component having a designated purpose:

- **controllers/**: Contains `DataController` classes for data processing.
- **models/**: Holds data models, such as `HumidityModel`, which are structured for sensor data.
- **graph/**: Manages graphical representation, particularly the `GraphView` responsible for line charts.
- **utils/**: Contains helper functions, including CSV parsing for test data loading.
- **qml/**: Includes QML files for the user interface, defining the main window, graphs, and layout components.

This modular design facilitates ease of development and testing, as well as integration with real sensors.

## Features

1. **Real-time Graph Visualization**: The application continuously updates graphs with sensor data (currently simulated), providing an up-to-the-second display.
2. **Gyro Animation**: A rotating rocket animation reflects real-time changes in pitch, yaw, and roll.
3. **Data Averaging and Current Value Calculation**: Real-time averages and current values are displayed, giving users detailed insights into telemetry data.

## Signal & Slot Mechanism

The application employs Qt’s signal-slot architecture to ensure fast data flow between the back-end (C++) and the front-end (QML):

- **Data Update**: When new data is added to a model (e.g., `HumidityModel`), signals like `modelChanged()` and `averageChanged()` are emitted.
- **Front-End Update**: The QML interface listens to these signals and triggers immediate updates to display the latest data visually.
- **Graph Refresh**: Each signal triggers the `GraphView` component to redraw graphs, ensuring the graphical representation remains accurate.

## Testing

The application supports two main testing methods:

1. **Random Data Generation**: Simulated real-time data using `QRandomGenerator` for testing continuous data handling and updating.
2. **CSV Data Import**: Historical data can be loaded from CSV files, enabling developers to validate performance with specific data sets. CSV testing helps to verify the signal-slot system and ensure data integrity across sessions.

## Future Improvements
1. **Sensor Integration**: Connect real sensors for real-world applications.
2. **Enhanced GUI**: Expand the interface to include advanced controls and customization options.
3. **Gyro Visualization Expansion**: Add visual indicators for additional data points, such as acceleration in multiple directions.


## Getting Started

### Dependencies

#### Windows

##### Required

1. **Git**  
   Install Git by running the following command in the command line:
   ```shell
   winget install git.git
   ```

   Or download Git for Windows and follow the instructions: [https://git-scm.com/downloads/win](https://git-scm.com/downloads/win)

   
2. **Qt**
   Download and install Qt from: [Download Qt Installer](https://www.qt.io/download-qt-installer-oss)


   During installation, make sure to:

- Select the correct version of Qt for your project (for example, **Qt 5.15** or **Qt 6**).
- Ensure **qmake** is included (this is usually included by default).

3. **Make**
   Download and install **GnuWin32 Make**: [https://www.gnu.org/software/make/#download](https://www.gnu.org/software/make/#download)  
   This will be required to run `make` commands in the project.

**Optional**  
**MinGW** (Minimalist GNU for Windows)  
Qt often comes with its own version of MinGW, so this is usually unnecessary if you install Qt from their website. If it's missing, you can download it here: [Download MinGW](https://sourceforge.net/projects/mingw/files/latest/download)

## Enviorment Variables
To ensure `qmake` and `make` work from the command line, we need to add the correct environment variables in Windows:

1. **Add Qt and qmake to PATH**
   After Qt is installed, add the path to `qmake` to `PATH`. Example of a common location:
   ```shell
   C:\Qt\<version>\<compiler>\bin
   ```
   For example: `C:\Qt\6.2.4\mingw81_64\bin` (check the folder where you installed Qt to find the correct path).
   
2. **Add Make to PATH**
   If you installed **GnuWin32 Make**, add `C:\Program Files (x86)\GnuWin32\bin` to the `PATH` variable.

**Note:** If you're unsure how to add environment variables, follow this guide: [Guide to Adding Windows PATH Environment Variable](https://helpdeskgeek.com/windows-10/add-windows-path-environment-variable/)

## Building the Project
Once all dependencies are installed, you can build the project using `qmake` and `make`.
1. **Generate Makefile with** `qmake`  
   Navigate to the project folder in the terminal and run:
   ```shell
   qmake
   ```
   
2. **Build the binary**  
   Run the following command to build the project:
   ```shell
   make
   ```

3. **Run the program**  
   Once the build is complete, you can run the generated executable from the same folder.


## Useful Resources
[Qt Documentation Topics](https://doc.qt.io/all-topics.html)
[QMake Manual (Qt 6)](https://doc.qt.io/qt-6/qmake-manual.html)

## Authors
- Emory Høiensahl
- Hamsa Hashi










   


