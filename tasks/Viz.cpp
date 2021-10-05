/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Viz.hpp"

using namespace tumvie2pocolog;

Viz::Viz(std::string const& name)
    : VizBase(name)
{
}

Viz::~Viz()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Viz.hpp for more detailed
// documentation about them.

bool Viz::configureHook()
{
    if (! VizBase::configureHook())
        return false;
    return true;
}
bool Viz::startHook()
{
    if (! VizBase::startHook())
        return false;
    return true;
}
void Viz::updateHook()
{
    VizBase::updateHook();
}
void Viz::errorHook()
{
    VizBase::errorHook();
}
void Viz::stopHook()
{
    VizBase::stopHook();
}
void Viz::cleanupHook()
{
    VizBase::cleanupHook();
}
