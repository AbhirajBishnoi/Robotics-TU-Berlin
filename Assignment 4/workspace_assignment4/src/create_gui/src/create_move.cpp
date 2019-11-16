#include <cstdlib>
#include <unistd.h>
#include <math.h>

//#include "rbo_create/Brake.h"
#include "rbo_create/Tank.h"

#include "create_gui/create_move.h"

using namespace std;

	CreateMove::CreateMove() {

		// make a persistent service connection!
		// this->brakeCreate = node.serviceClient<rbo_create::Brake>( "brake" );
		this->tankCreate = node.serviceClient<rbo_create::Tank>( "tank" );

		// wait for all services to exist (assure the create driver already started)
		// this->brakeCreate.waitForExistence();
		this->tankCreate.waitForExistence();
		move(0,0);
	}

	CreateMove::~CreateMove() {  }


	bool CreateMove::move(int req_right, int req_left) {
		// check if command differs from last successful command
		if( fabs(req_right - this->req_right_old) < 0.01 &&
                    fabs(req_left - this->req_left_old) < 0.01 ) {
			return true;
		}

		rbo_create::Tank srvArgs;
			//srvArgs.request.clear = true;
			srvArgs.request.right = req_right;
			srvArgs.request.left = req_left;


			if( this->tankCreate.call( srvArgs ) ) {

				this->req_right_old = req_right;
				this->req_left_old = req_left;
				return true;
			}
			else {
				ROS_ERROR("Failed to call service tank!");
				return false;
			}
	}

	bool CreateMove::stop() {

		return move(0,0);  //just command zero velocity

		// // check if command differs from last successful joystick command
		// 	irobot_create_2_1::Brake srvArgs;
		// 	srvArgs.request.brake = true;
		//
		// 	if( this->brakeCreate.call( srvArgs ) ) {
		// 		this->req_right_old = 0;
		// 		this->req_left_old = 0;
		// 		return true;
		// 	}
		// 	else {
		// 		ROS_ERROR("Failed to call service brake!");
		// 		return false;
		// 	}

	}
