#include <iostream>
#include <Dexterous_hands.h>

int main(int argc, const char ** argv)
{
    if(argc != 3)
    {
        std::cout << "Usage: dexhand021 <number1> <number2>" << std::endl;
        return 1;
    }

    short dist = (short)atoi(argv[1]);
    short prox = (short)atoi(argv[2]);

    Dexterous_hands hand;
    hand.start();

    hand.Degree_Control_mode(FingerID::LEFT_INDEX, Channel::Can0, JointMotor::ALL, prox, dist);

    hand.stop();

    return 0;
}

