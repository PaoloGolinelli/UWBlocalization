# UWB Localization System with Node-RED Dashboard

This repository contains the Node-RED flow, firmware binary, and project report for a real-time localization system developed using the **Qorvo QM33120WDK1** Ultra-Wideband (UWB) development kit.

## Overview

The system enables real-time tracking of mobile agents using distance and Angle-of-Arrival (AoA) measurements, visualized through a web-based dashboard developed in a Node-RED flow using the dashboard 2.0 library. The dashboard serves as a control panel, including tools for:

- Starting/stopping measurements
- Configuring measurement frequency
- Visualizing tracked positions in real-time, including quality metrics like RSSI and measurement rate
- Replaying and visualizing historic data in a given time window

The Node-RED flow contains all the source code for:
- Communicating the controls to the devices via USB
- Parsing incoming measurements
- Estimating the position of the tags via a Kalman Filter
- Storing data in a InfluxDB database
- Quering the database and visualizing the historic tracking positions

Additionally, the repository includes the MATLAB code used for analyzing the results of the MoCap performance evaluation campaign, along with the required dataset. 

For more information please refer to the provided project report

## Components

- `node_red_flow.json`: Full Node-RED flow including control, visualization, and data logging logic.
- `firmware/nRF52840DK-DW3_QM33_SDK_UCI-FreeRTOS.hex`: Official Qorvo-provided UCI firmware used for testing and calibration via Qorvo One graphic interface.
- `firmware/nRF52840DK-DW3_QM33_SDK_CLI-FreeRTOS.hex`: Official Qorvo-provided CLI firmware used for testing and data acquisition.
- `report/project_report.pdf`: Project documentation/report describing the implementation, architecture, and results.
- `evaluation/performance_analysis.m`: Matlab file used for evaluating the performance of the localization system against Motion Capture tracking
- `evaluation/mocap_data.mat`: Motion Capture data in matlab file format
- `evaluation/mocap_data.csv`: Motion Capture data in csv file format
- `evaluation/estpose.csv`: Localization data from Kalman Filter Estimates 
- `evaluation/rawpose.csv`: Localization data from Raw Measurements

## Disclaimer

The firmware binaries included here are the official Qorvo firmwares, redistributed for reproducibility and academic use. All rights belong to Qorvo.

## License

This project is intended for academic and educational purposes only.
