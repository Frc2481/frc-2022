// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/FunctionalCommand.h>
#include "subsystems/DriveSubsystem.h"
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <ioStream>

class AlignToTrussCommandGroup
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AlignToTrussCommandGroup> {

 private:
  bool m_foundTrussFirst;
 public:
  AlignToTrussCommandGroup(DriveSubsystem* drive){
    AddCommands(
      frc2::FunctionalCommand(
        [drive]{drive->Drive(.15_mps, 0_mps, 0_rad_per_s, false);},
        [drive]{},
        [drive](bool interrupted){drive->stop();
        std::cout << "found line " << drive->isTrussLineDetected() << " " << drive->isFloorLineDetected() << "\n";
        },
        [drive]{return drive->isTrussLineDetected() || drive->isFloorLineDetected();},
        {drive}),
      frc2::FunctionalCommand(
        [drive, this]{
          if(drive->isFloorLineDetected()){
                  drive->rotateAroundFloorSensor();
                  drive->Drive(0_mps, 0_mps, -0.2_rad_per_s, false); 
                  m_foundTrussFirst = false;  
                }else{
                  drive->rotateAroundTrussSensor();
                  drive->Drive(0_mps, 0_mps, 0.2_rad_per_s, false);
                  m_foundTrussFirst = true;
                }
                },
        [drive]{},
        [drive](bool interrupted){drive->stop();
                                  drive->rotateAroundMidpoint();
                                  std::cout << "other sensor found line " << drive->isTrussLineDetected() << " " << drive->isFloorLineDetected() << "\n";
                                  },
        [drive, this]{return (drive->isTrussLineDetected() && !m_foundTrussFirst); //||
                            //  (drive->isFloorLineDetected() && m_foundTrussFirst)
                             },
        {drive}
      ),
      frc2::FunctionalCommand(
        [drive]{drive->Drive(-.1_mps, 0_mps, 0_rad_per_s, false);},
        [drive]{},
        [drive](bool interrupted){drive->stop();
        std::cout << "both lines not found " << drive->isTrussLineDetected() << " " << drive->isFloorLineDetected() << "\n";
        },
        [drive]{return !drive->isTrussLineDetected() && !drive->isFloorLineDetected();},
        {drive}),
      frc2::FunctionalCommand(
        [drive]{drive->Drive(.075_mps, 0_mps, 0_rad_per_s, false);},
        [drive]{},
        [drive](bool interrupted){drive->stop();
        std::cout << "find one line " << drive->isTrussLineDetected() << " " << drive->isFloorLineDetected() << "\n";
        },
        [drive]{return drive->isTrussLineDetected() || drive->isFloorLineDetected();},
        {drive}),
      frc2::FunctionalCommand(
        [drive, this]{if(drive->isTrussLineDetected()){
                  drive->rotateAroundTrussSensor();
                  drive->Drive(0_mps, 0_mps, 0.15_rad_per_s, false);
                  m_foundTrussFirst = true;
                }else{
                  drive->rotateAroundFloorSensor();
                  drive->Drive(0_mps, 0_mps, -0.15_rad_per_s, false);
                  m_foundTrussFirst = false;
                }
                },
        [drive]{},
        [drive](bool interrupted){drive->stop();
                                  drive->rotateAroundMidpoint();
                                  std::cout << "other sensor find line " << drive->isTrussLineDetected() << " " << drive->isFloorLineDetected() << "\n";
                                  },
        [drive, this]{return (drive->isTrussLineDetected() && !m_foundTrussFirst) ||
                             (drive->isFloorLineDetected() && m_foundTrussFirst);},
        {drive}
       )
       
      //   frc2::FunctionalCommand(
      //   [drive, this]{if(drive->isTrussLineDetected()){
      //             drive->rotateAroundTrussSensor();
      //             drive->Drive(0_mps, 0_mps, 0.2_rad_per_s, false);
      //             m_foundTrussFirst = true;
      //           }else{
      //             drive->rotateAroundFloorSensor();
      //             drive->Drive(0_mps, 0_mps, -0.2_rad_per_s, false);
      //             m_foundTrussFirst = false;
      //           }
      //           },
      //   [drive]{},
      //   [drive](bool interrupted){drive->stop();
      //                             drive->rotateAroundMidpoint();},
      //   [drive, this]{return (drive->isTrussLineDetected() && !m_foundTrussFirst) ||
      //                        (drive->isFloorLineDetected() && m_foundTrussFirst);},
      //   {drive}
        // )
    );
  }
};
