# FRC_Code
All Seasons of FRC for the LIGHTSABERS Robots Team

# LIGHTSABERS Getting Started with FRC Robot Code

[![Gradle](https://github.com/wpilibsuite/allwpilib/actions/workflows/gradle.yml/badge.svg?branch=main)](https://github.com/wpilibsuite/allwpilib/actions/workflows/gradle.yml)
[![Java Documentation](https://img.shields.io/badge/documentation-java-orange)](https://github.wpilib.org/allwpilib/docs/development/java/)

Welcome to the LIGHTSABERS Robot Code Repository for the FRC 2024 Season. This Repositiory Contains the steps for installing the Coding Enviroment and loading in last year's FRC Robot Code.

- [Preliminaries with Git](#preliminaries-with-git)
- [Robot Home](#robot-home)
- [Zero to Robot](#zero-to-robot)
- [GitHub Revisited](#GitHub-Revisited)
- [Programming Basics](#programming-basics)
- [Creating a New Robot Project](#creating-a-new-robot-project)
- [3rd Party Libraries](#3rd-party-libraries)
- [Copy Last Years Robot Code](#last-years-robot-code)
- [Git Update](#git-update)
- [Git Again](#git-again)


# Preliminaries with Git

First of all, I'm assuming that you are reading this README.md file from the "main" branch on the LIGHTSABERS Web-based GetHub site (i.e. the remote Github repositiory).  The "main" branch is the branch that holds the "competition" code.  This code should always compile and be deployable to the Robot on a moments notice.  So, do not develop new code in the "main" branch.  The first thing to do is to create a new code development branch by clicking on the "branch" icon at the top of this repository.
![image](https://user-images.githubusercontent.com/54441806/205994006-44c873f8-bac8-4605-8b4d-bd86b5a21ced.png)
This will bring up a "branch" page.  You first click on "New Branch" button, then type in a name for the new Branch.  The name is usually representative of the type of "feature" that you plan to develop for the robot software (like "addGyro, or "updateSmartDashboard"), but for the rest of this discussion we will assume that you know ypur last name and the Branch Name you created is called "LastnameDev".  After selecting your development branch name, make sure that the "source" is set to <u><b> this year‘s current branch,</b></u> and then click "Create Branch".  
![Screenshot 2024-11-11 191707](https://github.com/user-attachments/assets/d04ee24d-b81a-487e-8316-c1dc96915934)
The resulting "Overview" page will then show that there is an Active branch called "LastnameDev".  You can click on the "LastnameDev" branch and you will see that it has the same files as were in the "main" branch.
![image](https://user-images.githubusercontent.com/54441806/205998157-9fa5980c-fed6-4cf2-befc-6f007a0b72eb.png)
  
At this point, I will assume that you are reading this README.md file from the "LastnameDev" branch.  You are now ready to setup your computer to receive the local repository of the FRC_Code.  If you don't already have a "Git" folder on your computer, create a new one called "GitHub" on your computer in the Public User Documents folder.  This folder will hold all of your Git projects.  For the rest of this discussion, it is assumed that your "Git" folder is called "GitHub", if you already have a "Git" folder that is named differently then substitute your folder's name appropriately in the discussions below.
  
"{Drive_Letter}:\Users\Public\Public Documents\GitHub"

Go to https://desktop.github.com/download/ to download Gethub Desktop (you need to download "GitHub Desktop" and install it before proceding).
   
On the GitHub Webpage (i.e. the remote Git repositiory), there is a green button called "Code" above the top right-hand corner of this README.  Right click on this button and the various options for "cloneing" this repository are offered.

![image](https://user-images.githubusercontent.com/54441806/206072601-60adb60e-19bf-4245-85d4-27e3497e33dd.png)
---------------------------------------------------------------------------------------------------------------
If you are using the GetHub Desktop, you will need to select the storage location you created above and make sure that the URL is pointing to the FRC_Code Online Repository before clicking Clone. 

When GitHub Desktop opens, then it may open to the "main" branch.  If this is the case then you must switch it to the "LastnameDev" branch.

![image](https://user-images.githubusercontent.com/54441806/206074279-9abeab05-935d-456a-8e2c-fa2f417320b7.png)

This repository now exists locally on your computer at:

"{Drive_Letter}:\Users\Public\Public Documents\GitHub\FRC_Code"


# Robot Home

The FRC_Code folder on your computer (local repository) has this README file, a license file, and another folder called "Robot_Code".  All Robot code projects for this year will be created in the folder called "2024_Robot_Code".  Each of these folders represents a WPILib Java project.  

# Zero to Robot

Starting with Step 2 of the Zero to Robot instructions (https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) install the WPILib Visual Studio Code libaries.  Since we are programming Java and we are not building the WPILib libraries, it is not necessary to perform the steps for the "Additional C++ Installation for Simulation".  When the instructions on this web page have been completed, the following will be installed:
  
Visual Studio Code - The supported IDE for 2019 and later robot code development. The offline installer sets up a separate copy of VS Code for WPILib development, even if you already have VS Code on your machine. This is done because some of the settings that make the WPILib setup work may break existing workflows if you use VS Code for other projects.

C++ Compiler - The toolchains for building C++ code for the roboRIO

Gradle - The specific version of Gradle used for building/deploying C++ or Java robot code

Java JDK/JRE - A specific version of the Java JDK/JRE that is used to build Java robot code and to run any of the Java based Tools (Dashboards, etc.). This exists side by side with any existing JDK installs and does not overwrite the JAVA_HOME variable

WPILib Tools - SmartDashboard, Shuffleboard, RobotBuilder, Outline Viewer, Pathweaver, Glass, SysID

WPILib Dependencies - OpenCV, etc.

VS Code Extensions - WPILib extensions for robot code development in VS Code

Continue to the next page in the instructions and select the "Visual Studio Code Basics" hyperlink.

# GitHub Revisited

Start "WPILib VS Code" application and open the folder called "{Drive_Letter}:\Users\Public\Public Doucuments\GitHub\FRC_Code\Robot_Code".  Before doing anything else select the "Source Control" icon on the left-hand wall of the WPILib Application.  This will change the left window pane to show GitHub Source control status and commands.  At the top of this pane, below "Source Control Repositories", you will see your current "local" repository (FRC_Code) and the current local "branch" ("main") for that repository.  Click on the branch and, at the bottom of the popup window, you will see your remote (on-line) development branch, which is called "Origin/LastnameDev
".  Select your remote development branch and the local branch will then change to be your development branch.  At this point any changes you make to the project are being made to your "development" branch code.  Your changes will not change the "main" branch competition code until you have completed your developement, tested it, and have approved it as ready for competition.

While you can make code changes, test your builds, deploy to, and even run the robot from the code in your Robot_Code folder; you should save often.  Since you code is also managed by software source control, you should periodically commit your changes to source control (Git) using the "Commit" button on the Source Control pane.  When you "Commit" the changes, you need to provide a comment above it to be stored as a record of what was was changed in this "Commit" operation.  So you should "Commit" often enough to keep the Comment for what was changed small.

Note: you do not need to Commit any binary files that are created when you Build, Deploy, add imports, or link libraries to your source code.  Select all of that are not your source files and on one of them select the revert button -- the curved arrow curling back on itself.  When only your source code files are shown below the "Commit" button, then you can continue with the "Commit" operation.

After you Commit your changes they are stored in your local Git repository.  The "SYNC" button with the up arrow (it replaces the "Commit" button after you Commit the changes) will update the remote repository on the GitHub Website with your new changes.  At that point other team members can look at the changes you have made on the website. 

You can then open one of the java source files in the project and examine its code.  This code probably will not build using the WPILib on your computer because you probably have not added 3rd party libaries required to run the devices that this version of the Robot_Code is using.  Read the FRC documation on installing [3rd party libraries](#3rd-party-libraries) found at:  https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html#rd-party-libraries.

Any files that you add or modify in this folder are added to the files to "Commit" to Git.  You can add these files to the "GitIgnore" file by right clicking on each and chosing the "GitIgnore" option.

The Robot_Code in the Local Repository is now different from the Remote Repository on the Web.  For example: In the directory above, if you open the README.md file in a text editor (ex. wordpad), change the spelling of a word, and then save the file; GitHub Desktop will change to look like this:

![image](https://user-images.githubusercontent.com/54441806/206082745-758bfe6e-c9ea-4745-9dae-d1e5575a19e8.png)

Even though the file has changed, it is not stored in Git until a "Commit" is completed and has a relevant change comment. After the change has been "Committed" to the Local Repository, it can be backed-up, or sync'ed, to the Remote Repository on the Web by clicking on the "push" button a the top of the GitHub Window.  This whole example shows that source control can be handled by GitHub Desktop if necessary, but the Commit and Push (sync) operations are normally done in the WPILib VS Code Source Control Pane.   

![image](https://user-images.githubusercontent.com/54441806/206085043-25b620a6-297c-477d-adf6-412e7a08d93c.png)

Now you can create or open Robot project files in the local repository using the Visual Studio Code editor and when you get all done you can commit them to Git storage and back them up (or sync them) to the Git remote storage on the Web. 


# Programming Basics

The "Visual Studio Code Basics" hyperlink redirects to a webpage called "Visual Studio Code Basics and WPILib Extension" (https://docs.wpilib.org/en/stable/docs/software/vscode-overview/vscode-basics.html#visual-studio-code-basics-and-the-wpilib-extension), which is part of the "Programming Basics" Tutorial.

Read this page to get familiar with how to call the WPILib command from the command set list.  Click the "Next" button a the bottom of the page to go to next page where a description of each of the WPILib commands is found.  Click the "Next" button to to to the next page, which starts with a discussion on Robot Program and how a Robot Base Class is selected.  Pay particular attention to the "Command Robot Template". 


# Creating a New Robot Project
  
Eventually, a section called "Creating a New WPILib Project" is found toward the middle of the the Robot Program Page (https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html).  Unless you are going to start from scratch and delete the 2024_Robot_Code folder, you should not need to create a "new" Project.  But if creating a new project from scratch is your intention (The 2024_Robot_Code had to come from somewhere.  Right?), then perform the tasks stated in Visual Studio Code documetation above. 

While doing these tasks, a "New Project Creator Window" will pop up.  Select the following on the first row:

Project Type:  Template
Language:      java,
Base:          Command Robot

Other items on the creator page are filled in as:
Base Folder:  {Drive Letter}:/Users/Public/GitHub
Use the "Select a new project folder" button to graphically select the Base Folder

Project Name: {New_Project_Name},
Team Number:  3660.

Then click on the "Generate Project" button.

Visual Studio will now have a project created in the FRC_Code Folder called "{New_Project_Name}".

After creating the project in Visual Studio Code, continue reading to the webpage instructions to end of the page and click the "Next" button to continue to "libraries".

# 3rd Party Libraries

At this point in the process, you should be on the 3rd Party Libraries Page of the Web instructions(https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html). The necessary libaries for running all of the devices on the Robot must be installed before the 2024_Robot_Code will compile (i.e. build) and run properly.  Read down to "Managing VS Code Libraries".  In VS Code, use the "Manage Vendor Libraries > Manage Current Libraries" to show you that:

WPILib-New-Commands

library is already part of your project.  Note: If you libraries are project specific.  If you closed the project, then it must be reopened to show the libraries currently in use.

Pay attention to the "command line" option for installing vendor libraries.  Three-quarters of the way down this page is a list of common vendors and their web pages.

Add the Rev library and the Phenoix Library to your project by:

1. Open a terminal in VS Code from the "View > Terminal" tab.

2. Navigate the directory "<Drive Letter>:\Users\Public\Documents\GitHub\FRC_2022-2023\Robot_Code\Last_Year"

3. Give the following commands at the terminal command prompt:

> ./gradlew vendordep --url=https://software-metadata.revrobotics.com/REVLib.json

>./gradlew vendordep --url=https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json

Note: these website addresses were valid for the 2022 season.

In VS Code, use the "Manage Vendor Libraries > Manage Current Libraries" to show you 3 libraries are part of the project:

CTRE-Pheonix
RevLib
WPILib-New-Commands

# Last Years Robot Code

Last year's robot code is in the 2023_Robot_Code folder.  If you open the folder for 2023_Robot_Code folder in WPILib VS Code, you can examine that project.  That project will have many errors due to the fact that the 3rd party libraries have changed this year.  If you want it to build, code changes are needed to make it compatable with the 2024 3rd party libraries. 

# Git Update

Remember to select your developement branch for Source Control and to save and "Commit" often.  Push the change to the remote repository when finished by clicking on the "Push Origin" button on the top (right side) of the GitHub Desktop window or by sync'ing in the WPILib Source Control pane. Before "pull"ing new changes into the "main" branch, the Drive Team must be informed that the changes are being added to the competition code.  When informing the Drive Team, be sure to describe how the changes affect the operation of the Robot.

## Check That "LastnameDev" is Ready to Be Delivered
After you have tested your development code and you feel that the code is good enough for competition, the next step is to check for any changes that have been made on the "main" branch while you were devloping your code.  To do this, make a "pull" request in GitHub Web.  The request is to pull the changes from the "main" branch into your "LastnameDev" Branch.  If there are no changes identified in this pull request, then "LastnameDev" branch is ready to be delivered.  Skip to [Deliver Code to Main Branch](##Deliver_Code_to_Main_Branch).

If the "pull" request from the "main" branch to "LastnameDev" branch shows that changes have been made, then another developer has delivered changes to the competition code since "LastnameDev started implementing and testing. To prevent overwriting delivered changes, all of these changes from this request must be accepted into the "LastnameDev" branch.  When these changes are accepted, it is possible that they will conflict with, or break, the code that is currently in the "jdoeDev" branch.  Check that the code in "LastnameDev" still builds and test it before repeating the this process.  Repeat this process until the "LastnameDev" branch ready to be delivered, as described above.

## Deliver Code to Main Branch
After determining that the "LastnameDev" branch is ready to be delivered, a "pull" request to pull the changes from "LastnameDev" branch into the "main" branch can be made and there should be no conflict.  After you have delivered (merged) the code from the "LastnameDev" Branch into the Main, you should test the "main" code on the robot.  If it works as planned, then the "LastnameDev" Branch can be deleted.

# Git Again

If you decide that you want to develop another feature of the Robot, the process is to just create a new "feature" branch and start working the the new Branch.
