/******************************************************************************/
/*!
\file		Main.cpp
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Program entry point

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#include <pch.h>
#include <io.h>
#include <fcntl.h>
//#include <iostream>

std::unique_ptr<Engine> engine;

// Indicates to hybrid graphics systems to prefer the discrete part by default
extern "C"
{
    __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}

void create_debug_console()
{
    FILE *current = nullptr;
    AllocConsole();
    freopen_s(&current, "CONOUT$", "w", stdout);
}

// Entry point
int WINAPI wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{
    create_debug_console();

    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    if (!DirectX::XMVerifyCPUSupport())
        return 1;

    HRESULT hr = CoInitializeEx(nullptr, COINITBASE_MULTITHREADED);
    if (FAILED(hr))
        return 1;

    // create and initialize the engine
    engine = std::make_unique<Engine>();

    if (engine->initialize(hInstance, nCmdShow) == false)
    {
        std::cout.flush();
        if (engine->should_pause_on_exit() == true)
        {
            system("pause");
        }
        return -2;
    }

    engine->finalize_project();

    // Main message loop
    MSG msg = {};
    while (WM_QUIT != msg.message)
    {
        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        else
        {
            engine->tick();
        }
    }

    // free all resources used by the engine
    engine->shutdown();

    CoUninitialize();

    if (engine->should_pause_on_exit() == true)
    {
        system("pause");
    }

    return (int) msg.wParam;
}
