// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IndexerSubsystem.h"

IndexerSubsystem::IndexerSubsystem() :m_isIndexerOn(false)
{
m_indexerMotor = new VictorMotorController(VictorIDs::kIndexerRollerMotorID, "IndexerMotor");
}

void startIndexer(){

}

void stopIndexer(){

}

bool isIndexerOn(){
    return true;  //FIX THIS MADDOX PERIOD!
}

// This method will be called once per scheduler run
void IndexerSubsystem::Periodic() {}
