/*
 * @file motors.hpp
 *
 * @author Ximena Patricia García Magdaleno
 * 
 * @brief Header file for the bno Class.
 *
 * @version 0.1
 * 
 * @date 2026-01-12
 */

#ifndef SYSTEM_HPP
#define SYSTEM_HPP

class System
{
public:
    System() = default;
    virtual void begin() = 0;
    virtual void update() = 0;
    virtual void setState(int state) = 0;
};

#endif // SYSTEM_HPP