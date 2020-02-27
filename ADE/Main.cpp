#include <ADE/DracoWrapper.hpp>
#include <iostream>

void StartControls(DracoWrapper& w) {
    bool live = true;
    while (live) {
        char nextInput;
        std::cout << "Enter a move command or c to exit\n";
        std::cin >> nextInput;
        switch (nextInput) {
            case 'w':
                std::cout << "Walking Forward!\n";
                w.SetWalkYCommand(1.0);
                break;
            case 'a':
                std::cout << "Walking Left!\n";
                w.SetWalkXCommand(-1.0);
                break;
            case 's':
                std::cout << "Walking Back!\n";
                w.SetWalkYCommand(-1.0);
                break;
            case 'd':
                std::cout << "Walking Right!\n";
                w.SetWalkXCommand(1.0);
                break;
            case 'c':
                std::cout << "Exiting!\n";
                live = false;
                break;
            default:
                break;
        }
    }
}

int main(int argc, char** argv) {
    DracoWrapper w;
    w.Initialize();
    StartControls(w);
    w.Shutdown();
    return 0;
}