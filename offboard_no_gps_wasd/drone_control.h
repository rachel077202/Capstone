#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>

// --- Constants ---
constexpr float       HOVER_THRUST   = 0.55f;
constexpr float       MAX_PITCH_DEG  = 15.0f;
constexpr float       MAX_ROLL_DEG   = 15.0f;
constexpr float       THRUST_MIN     = 0.30f;
constexpr float       THRUST_MAX     = 0.80f;
constexpr int         LOOP_HZ        = 20;
constexpr const char* CONNECTION_URL = "udpin://192.168.4.1:14550";

// --- Shared attitude state ---
// Written by input/control thread, read by sender thread
struct SharedState {
    std::atomic<float> roll_deg   {0.0f};
    std::atomic<float> pitch_deg  {0.0f};
    std::atomic<float> yaw_deg    {0.0f};
    std::atomic<float> thrust     {HOVER_THRUST};
    std::atomic<bool>  running    {true};
    std::atomic<bool>  offboard_ok{false};
};

// --- Drone context ---
// Owns all objects that must outlive initialize()
struct DroneContext {
    SharedState        state;
    std::thread        sender;
    mavsdk::Action     action;
    mavsdk::Offboard   offboard;

    DroneContext()                               = default;
    DroneContext(const DroneContext&)            = delete;
    DroneContext& operator=(const DroneContext&) = delete;
};

// --- Function declarations ---
bool initialize(const std::string& url, DroneContext& ctx);
void droneControl(DroneContext& ctx, float pitch, float yaw, float roll, float thrust);
void shutDown(DroneContext& ctx);