/*
 * @file bno.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Source file for the sorter class.
 * @version 0.1
 * @date 2026-03-27
 */

#include "sorter.hpp"

SORTER::SORTER() {}

void SORTER::startStateTime()
{
    if (now == 0)
    {
        now = millis();
    }
}

void SORTER::moveLeft()
{
    if(boxLeft){
        
    }
}

void SORTER::moveRight()
{
}

void SORTER::separate(bool warmBean)
{

    if (warmBean)
    {
        separateS.write(90);
    }
    else
    {
        separateS.write(0);

    }
}

void SORTER::box(SORTER box_, bool left)
{
    if (left)
    {
        box_.moveLeft();
    }
    else
    {
        box_.moveRight();
    }
}
