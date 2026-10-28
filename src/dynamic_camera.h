#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // TODO: (HINTS/outline)
        // collect statistics on the player position and velocity in (normalized, if you like)
        // window coordinates. Compute lookAhead distance in world coords, adjusting for
        // if the player is moving left or right.
        // Now update lookAhead state using an EMA over its previous state and the lookAhead world
        // vector. Then determine if the player is in the dead zone, soft rejection zone, or hard
        // rejection zone. For each, perform EMA on the delta of the camera displacement (ignoring
        // lookahead) appropriately. Next,
        // set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        // Finally update cameraUnadjustedCenter, but without lookahead.
        // std::cout << "Position of player is now: " << trackedPosWorld <<"\n";
        // std::cout << "Velocity of player is now: " << trackedVelocityWorld <<"\n\n";

        // std::cout << "Position of player is now: " << camera.worldToWindowPercent(trackedPosWorld) <<"\n";
        // std::cout << "Velocity of player is now: " << trackedVelocityWorld/50.f<<"\n";
        //lookAhead = camera.windowToWorldDist(camera.worldToWindow(trackedPosWorld)).x;
        //V
        camera.setCameraCenter(cameraUnadjustedCenter);
        float playerPct = camera.worldToWindowPercent(trackedPosWorld).x;

        if (!deadZoneX.contains(playerPct)){
            if(playerPct - deadZoneX.max > 0 && playerPct - softZoneX.max <= 0){
                cameraUnadjustedCenter += camera.windowPercentToWorldDist({playerPct - deadZoneX.max, 0.f})*alpha;
            }else if(playerPct - softZoneX.max > 0){
                cameraUnadjustedCenter += camera.windowPercentToWorldDist({playerPct - softZoneX.max, 0.f});
            }else if(playerPct - deadZoneX.min < 0 && playerPct - softZoneX.min >= 0){
                cameraUnadjustedCenter += camera.windowPercentToWorldDist({playerPct - deadZoneX.min , 0.f})*alpha;
            }else if(playerPct - softZoneX.min < 0){
                cameraUnadjustedCenter += camera.windowPercentToWorldDist({playerPct - softZoneX.min, 0.f});
            }
        }
        // else{
        //     // std::cout << "in dead zone: " <<"\n'\n";
        // }   
        float lookaheadWorld = camera.windowPercentToWorldDist({lookAhead, 0.f}).x;
        float target = 0.f;
        if (trackedVelocityWorld.x > 0){
            target = 1*lookaheadWorld;
        }
        else if (trackedVelocityWorld.x < 0){
            target = -1*lookaheadWorld;
        }

        lookaheadState += alpha * (target - lookaheadState);


        camera.setCameraCenter(cameraUnadjustedCenter + sf::Vector2f{lookaheadState, 0.f});
    }


    //     if(camera.worldToWindow(trackedPosWorld).x >= camera.mWindowSize.x / 3.f && camera.worldToWindow(trackedPosWorld).x <= camera.mWindowSize.x * 2.f / 3.f){
    //         if(startup == true){
    //             //camera.setCameraCenter({camera.mWindowSize.x*0.5f, WINDOW_HEIGHT * 0.5f});
    //             startup = false;
    //         }else{
    //             //camera.translateWindow({0.f , 0.f});      
    //         }
    //     }
    //     else if ((camera.worldToWindow(trackedPosWorld).x < camera.mWindowSize.x * deadZoneX) || (camera.worldToWindow(trackedPosWorld).x > camera.mWindowSize.x * 4.f * deadZoneX)){
    //         //float cameraOffset = camera.mWindowSize.x * HARD_LIMIT_PERCENT;
    //         if(camera.worldToWindow(trackedPosWorld).x < camera.mWindowSize.x * HARD_LIMIT_PERCENT) {
    //             //camera.translateWindow({camera.worldToWindow(trackedPosWorld).x - cameraOffset , camera.mWindowSize.x *0.f});
    //         }
    //         else if( camera.worldToWindow(trackedPosWorld).x > camera.mWindowSize.x * 4.f * HARD_LIMIT_PERCENT){
    //             //camera.translateWindow({camera.worldToWindow(trackedPosWorld).x - cameraOffset*4.f , camera.mWindowSize.x *0.f});
    //         }
    //     }
    //     else if ((camera.worldToWindow(trackedPosWorld).x < camera.mWindowSize.x *SOFT_LIMIT_PERCENT) || (camera.worldToWindow(trackedPosWorld).x > camera.mWindowSize.x * 2.f * SOFT_LIMIT_PERCENT)){
    //         //float cameraOffset = camera.mWindowSize.x * SOFT_LIMIT_PERCENT;
    //         if(camera.worldToWindow(trackedPosWorld).x < camera.mWindowSize.x * SOFT_LIMIT_PERCENT) {
    //             //camera.translateWindow({(camera.worldToWindow(trackedPosWorld).x - cameraOffset) * SOFT_CAMERA_ALPHA, camera.mWindowSize.x *0.f });
    //         }
    //         else if( camera.worldToWindow(trackedPosWorld).x > camera.mWindowSize.x * 2.f * SOFT_LIMIT_PERCENT){
    //             //camera.translateWindow({(camera.worldToWindow(trackedPosWorld).x - cameraOffset*2.f)* SOFT_CAMERA_ALPHA , camera.mWindowSize.x *0.f});
    //         }
    // }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords

};

#endif // DYNAMIC_CAMERA_H
