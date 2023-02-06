#pragma once

// Include all node headers in this file

// Example Control Flow Nodes
#include "ControlFlow/C_ParallelSequencer.h"
#include "ControlFlow/C_RandomSelector.h"
#include "ControlFlow/C_Selector.h"
#include "ControlFlow/C_Sequencer.h"

// Student Control Flow Nodes
#include "ControlFlow/C_LeftFirst.h"


// Example Decorator Nodes
#include "Decorator/D_Delay.h"
#include "Decorator/D_InvertedRepeater.h"
#include "Decorator/D_RepeatFourTimes.h"

// Student Decorator Nodes


// Example Leaf Nodes
#include "Leaf/L_CheckMouseClick.h"
#include "Leaf/L_Idle.h"
#include "Leaf/L_MoveToFurthestAgent.h"
#include "Leaf/L_MoveToMouseClick.h"
#include "Leaf/L_MoveToRandomPosition.h"


// Student Leaf Nodes
#include "Leaf/L_AvoidFromEnemy.h"
#include "Leaf/L_ChaseSurvivor.h"
#include "Leaf/L_ChaseAnimal.h"
#include "Leaf/L_ZombieCollisionCheck.h"
#include "Leaf/L_SpawnNewZombie.h"
#include "Leaf/L_ChaseZombie.h"
#include "Leaf/L_KillerIncreaseBodySize.h"
#include "Leaf/L_Zombie2Patrol.h"
#include "Leaf/L_SurvivorCollisionCheck.h"