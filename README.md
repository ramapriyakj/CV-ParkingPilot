# Setup

1. Make a backup of the existing *semesterprojekt* folder in  *..\Documents\Cassandra\11.8\plugins*
2. Clone this repository to *..\Documents\Cassandra\11.8\plugins\semesterprojekt*

# Usage

To use any of the provided stations or test your own code, open the file *semesterprojekt.sln*, compile in release mode and click the run button in Visual Studio. In Cassandra the stations are located under the *semesterprojekt* entry.

# Graphs

In the folder *..\Documents\Cassandra\11.8\plugins\semesterprojekt\runtime\graphs* you will find basic graphs which serve as a starting point for each of the groups (lidar, radar, camera and fusion)

# Setup Camera Station

1. Download https://github.com/Phylliida/orbslam-windows/raw/master/Vocabulary/ORBvoc.txt.tar.gz
2. Extract ORBvoc.txt into cassandra/bin/Release
3. Copy runtime/extra/hella.yaml and runtime/extra/opencv_world320.dll into cassandra/bin/Release
4. Don't worry if loading the camera station freezes cassandra for ~30secs, the initialization takes a long time, but localization is quick