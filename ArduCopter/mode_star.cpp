#include "Copter.h"

#if MODE_STAR_ENABLED == ENABLED

/*
* Init and run calls for "star" flight mode
*/

// initialise star controller
bool ModeStar::init(bool ignore_checks)
{
	bool takingoff = takeoff.running();
    bool armed = motors->armed();
    fprintf(stderr,"[%s:%d] Entering... ignore_checks=%d, armed=%d, takingoff=%d\n",
        __FUNCTION__, __LINE__, ignore_checks, armed, takingoff);

    //int16_t numCmd = mission.num_commands();

    if (ignore_checks) {
        _mode = Auto_Loiter;

        if (!armed) {
            fprintf(stderr,"[%s:%d] Leaving w/ return false. Disarmed !!\n",
                __FUNCTION__, __LINE__);
            return false;
		}

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw.mode() == AUTO_YAW_ROI) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // clear guided limits
        copter.mode_guided.limit_clear();
    }

	origin = wp_nav->get_wp_origin();
    fprintf(stderr,"[%s:%d] origin=(%f, %f, %f)\n",
        __FUNCTION__, __LINE__, origin.x, origin.y, origin.z);

	//Location homeLoc = copter.current_loc;
	startLoc = copter.current_loc;
    startLoc.alt += (startLoc.alt + 10 * 100);  // cm
    fprintf(stderr,"[%s:%d] homeLoc=(%f, %f, %f)\n",
        __FUNCTION__, __LINE__, startLoc.lat * 1e-7, startLoc.lng * 1e-7, startLoc.alt * 0.01);

    pilot_yaw_override = false;

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    copter.circle_nav->init();
#if 0
    // initialize's loiter position and velocity on xy-axes from current pos and velocity
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialise position_z and desired velocity_z
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise waypoint state
    stage = STORING_POINTS;
    dest_A.zero();
    dest_B.zero();
#endif

    fprintf(stderr,"[%s:%d] Leaving w/ return true.\n", __FUNCTION__, __LINE__);
    return true;
}

// run the star controller
// should be called at 100hz or more
void ModeStar::run()
{
    bool takingoff = takeoff.running();
    //fprintf(stderr,"[%s:%d] Entering... mode=%d, takingoff=%d\n",
    //    __FUNCTION__, __LINE__, _mode, takingoff);

    if (!takingoff) {
        takeoff_start(startLoc);
    }

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // get pilot's desired yaw rate (or zero if in radio failsafe)
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        pilot_yaw_override = true;
    }

    // get pilot desired climb rate (or zero if in radio failsafe)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = copter.get_surface_tracking_climb_rate(target_climb_rate);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    copter.circle_nav->update();

    // call attitude controller
    if (pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(copter.circle_nav->get_roll(),
                                                                      copter.circle_nav->get_pitch(),
                                                                      target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(copter.circle_nav->get_roll(),
                                                           copter.circle_nav->get_pitch(),
                                                           copter.circle_nav->get_yaw(), true);
    }

    // update altitude target and call position controller
    // protects heli's from inflight motor interlock disable
    if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::GROUND_IDLE && !copter.ap.land_complete) {
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
    } else {
        pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    }
    pos_control->update_z_controller();
#if 0
    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (is_disarmed_or_landed() || !motors->get_interlock() ) {
        //zero_throttle_and_relax_ac(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // auto control
    if (stage == AUTO) {
        // if vehicle has reached destination switch to manual control
        if (reached_destination()) {
            AP_Notify::events.waypoint_complete = 1;
            return_to_manual_control(true);
        } else {
            auto_control();
        }
    }

    // manual control
    if (stage == STORING_POINTS || stage == MANUAL_REGAIN) {
        // receive pilot's inputs, do position and attitude control
        manual_control();
    }
#endif
}

uint32_t ModeStar::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target();
}

int32_t ModeStar::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target();
}

bool ModeStar::start_command(const AP_Mission::Mission_Command& cmd)
{
    return true;
}

bool ModeStar::verify_command(const AP_Mission::Mission_Command& cmd)
{
    return true;
}

void ModeStar::exit_mission()
{
}

// do_guided - start guided mode
bool ModeStar::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (copter.control_mode != GUIDED && !(copter.control_mode == AUTO && mode() == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
        {
            // set wp_nav's destination
            Location dest(cmd.content.location);
            return copter.mode_guided.set_destination(dest);
        }

        case MAV_CMD_CONDITION_YAW:
            do_yaw(cmd);
            return true;

        default:
            // reject unrecognised command
            return false;
    }

    return true;
}

// auto_rtl_start - initialises RTL in AUTO flight mode
void ModeStar::rtl_start()
{
    _mode = Auto_RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(true);
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
void ModeStar::takeoff_start(const Location& dest_loc)
{
    _mode = Auto_TakeOff;

    Location dest(dest_loc);

    if (!copter.current_loc.initialised()) {
        // vehicle doesn't know where it is ATM.  We should not
        // initialise our takeoff destination without knowing this!
        return;
    }

    // set horizontal target
    dest.lat = copter.current_loc.lat;
    dest.lng = copter.current_loc.lng;

    // get altitude target
    int32_t alt_target;
    if (!dest.get_alt_cm(Location::AltFrame::ABOVE_HOME, alt_target)) {
        // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
        AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
        // fall back to altitude above current altitude
        alt_target = copter.current_loc.alt + dest.alt;
    }

    // sanity check target
    if (alt_target < copter.current_loc.alt) {
        dest.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
    }
    // Note: if taking off from below home this could cause a climb to an unexpectedly high altitude
    if (alt_target < 100) {
        dest.set_alt_cm(100, Location::AltFrame::ABOVE_HOME);
    }

    // set waypoint controller target
    if (!wp_nav->set_wp_destination(dest)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();
}

#endif // MODE_STAR_ENABLED == ENABLED
