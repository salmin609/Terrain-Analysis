/******************************************************************************/
/*!
\file		Messenger.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Simple callback based message system

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once
#include "Messages.h"
#include "Misc/NiceTypes.h"
#include <vector>
#include <array>

// simple messaging system, doesn't support removal, mostly for engine comms
class Messenger
{
public:
    static void listen_for_message(Messages type, Callback callback);
    static void send_message(Messages type);

    static void clear_all_listeners();
private:
    static std::array<std::vector<Callback>, static_cast<size_t>(Messages::NUM_ENTRIES)> inboxes;
};