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
    dest_A = Location(
			startLoc.lat + 901,		// deg * 1e7
			startLoc.lng,			// deg * 1e7
			startLoc.alt + 500,		// cm
			Location::AltFrame::ABOVE_HOME);
    dest_B = Location(
			startLoc.lat - 1631,	// deg * 1e7
			startLoc.lng - 643,		// deg * 1e7
			startLoc.alt - 500,		// cm
			Location::AltFrame::ABOVE_HOME);
    dest_C = Location(
			startLoc.lat + 1008,	// deg * 1e7
			startLoc.lng + 1683,	// deg * 1e7
			startLoc.alt,			// cm
			Location::AltFrame::ABOVE_HOME);
    dest_D = Location(
			startLoc.lat,			// deg * 1e7
			startLoc.lng - 2079,	// deg * 1e7
			startLoc.alt,			// cm
			Location::AltFrame::ABOVE_HOME);
    dest_E = Location(
			startLoc.lat - 1008,	// deg * 1e7
			startLoc.lng + 1683,	// deg * 1e7
			startLoc.alt - 500,		// cm
			Location::AltFrame::ABOVE_HOME);

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

	_mode = Auto_WP;
	wp_nav->set_wp_destination(dest_A);

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
        //takeoff_start(startLoc);
    }

	switch (_mode) {

		case Auto_TakeOff:
			takeoff_run();
			break;

		case Auto_WP:
			wp_run();
			break;

		case Auto_Land:
			land_run();
			break;

		case Auto_RTL:
			rtl_run();
			break;

		case Auto_Loiter:
			loiter_run();
			break;

		default:
			break;
	}
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

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeStar::wp_start(const Location& dest_loc)
{
    _mode = Auto_WP;

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
void ModeStar::takeoff_run()
{
    auto_takeoff_run();
    if (wp_nav->reached_wp_destination()) {
        const Vector3f target = wp_nav->get_wp_destination();
        wp_start(target);
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeStar::wp_run()
{
    //fprintf(stderr,"[%s:%d] Entering...\n", __FUNCTION__, __LINE__);

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        wp_nav->wp_and_spline_init();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

void ModeStar::land_run()
{
}

void ModeStar::rtl_run()
{
}

void ModeStar::loiter_run()
{
}

#endif // MODE_STAR_ENABLED == ENABLED
