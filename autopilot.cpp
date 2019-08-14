#include "autopilot.h"



class Yaw:public Behaviour
{
    float targetY;
    float tolY;
    float yawR;

    void ruleBase()
    {
        posTarget.type_mask=0b0000001111111000;

        while(threadRunning && !stopFlag)
        {
            posTarget.yaw=targetY;
            posTarget.yaw_rate=yawR;

            //cout<<param.cAtt.yaw<<" "<<abs(targetY-param.cAtt.yaw)<<endl;
            if(abs(targetY-param.cAtt.yaw) <= tolY)
            {
                cout<<"\nReached target yaw"<<std::flush;
                taskCompleted=true;
                break;
            }
        }
    }

public:
    Yaw(QuadParam &q, mavlink_set_position_target_local_ned_t &posT, float yaw, float ty, float rate):Behaviour(q,posT)
    {
        cout<<"\nYaw started."<<std::flush;
        name="Yaw";
        targetY = yaw;
        tolY = ty;
        yawR = rate;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&Yaw::ruleBase,this);
        }
    }

    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~Yaw()
    {
        cout<<"\nYaw stopped."<<std::flush;
        stop();
    }
};

class Land:public Behaviour
{
    void ruleBase()
    {

        posTarget.z=-0.4;
        posTarget.x=param.cPos.x;
        posTarget.y=param.cPos.y;
        posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;

        while(threadRunning && !stopFlag)
        {
            while(abs(param.cPos.z-posTarget.z) >= 0.05 && !stopFlag)
            {
                usleep(50000);
            }

            if(!stopFlag)
            {
                posTarget.vz=+0.2;
                posTarget.vx=0;
                posTarget.vy=0;
                posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY;

                //Hopefully hits the ground by now
                if(param.cPos.z > -0.12)
                {
                    taskCompleted=true;
                    break;
                }
            }

            if(stopFlag)
            {
                posTarget.vz=0;
                posTarget.vx=0;
                posTarget.vy=0;
                posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY;
                break;
            }
        }
    }

public:
    Land(QuadParam &q, mavlink_set_position_target_local_ned_t &posT):Behaviour(q,posT)
    {
        cout<<"\nLand started."<<std::flush;
        name="Land";
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&Land::ruleBase,this);
        }
    }

    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~Land()
    {
        cout<<"\nLand stopped."<<std::flush;
        stop();
    }
};


class Takeoff:public Behaviour
{
    float targetZ;
    float targetY;
    float targetTol;

    void ruleBase()
    {
        int task=1;
        int cnt=0;

        uint64_t tHoverStarted = -1;

        mavlink_set_position_target_local_ned_t objPosTarget;

        while(threadRunning)
        {
            //taskCompleted=true;
            usleep(25000);

            if(!taskCompleted)
            {
                //taskCompleted=true; //Debug purposes
                //takeoff to height specified
                if(task==1)
                {
                    posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_VELOCITY;

                    //printf("\ncP: %f, Sonar: %f, err: %f",param.cPos.z,(targetZ-param.cPos.z));
                    //cout<<"\nCurpos error: "<<param.fFlowDist;
                    posTarget.vz=-1.5;
                    posTarget.vx=0;
                    posTarget.vy=0;

                    if(param.cPos.x!=0)
                    {
                        task==2;
                    }

                    if(abs(targetZ-param.cPos.z) <= targetTol)
                    {
                        posTarget.vz=-0.05;
                        cout<<"\nReached takeoff height"<<std::flush;
                        task=2;
                    }
                }

                if(task==2)
                {
                    if(param.iAtt.yaw==-7)
                    {
                        if(param.cPos.x!=0 && param.cPos.y!=0)
                        {
                            param.iPos.x=param.cPos.x;
                            param.iPos.y=param.cPos.y;
                            param.iPos.z=targetZ;
                            param.timestamp.iPos=getTimeUsec();

                            param.iAtt.yaw=targetY;
                            param.timestamp.iAtt=getTimeUsec();

                            printf("\nGot an initial position of (%f,%f,%f) and yaw angle of %f",param.iPos.x,param.iPos.y,param.iPos.z,param.iAtt.yaw);

                            posTarget.x=param.iPos.x;
                            posTarget.y=param.iPos.y;
                            posTarget.z=param.iPos.z;
                            posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;

                            task=3;
                        }
                        else
                            cout<<"\nNo local positions available"<<endl;
                    }
                    else
                    {
                        posTarget.x=param.cPos.x;
                        posTarget.y=param.cPos.y;
                        posTarget.z=targetZ;
                        posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;
                        task=3;
                    }
                }

                //yaw to desired angle
                if(task==3)
                {
                    cout<<"\nYawing to initial angle"<<endl;
                    objPosTarget=posTarget;
                    Yaw obj(param,objPosTarget,targetY,0.05,0.05);
                    obj.start();
                    while(!obj.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;
                        usleep(1e5);
                    }
                    obj.stop();
                    task=4;
                }

                //hover after takeoff
                if(task==4)
                {
                    //cout<<"\nHover for 5 seconds"<<endl;
                    posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;

                    if(tHoverStarted == -1)
                    {
                        if(abs(param.cPos.z-targetZ-0.3) <= targetTol+0.3)
                        {
                            cout<<"\nHover for 5 seconds"<<endl;
                            tHoverStarted = getTimeUsec();
                        }
                    }

                    if(param.imgData.homebase.mX != -1)
                    {
                        float lx,ly;
                        bodyToNED(param.imgData.homebase.mX,param.imgData.homebase.mY,param.cAtt.yaw,lx,ly);
                        posTarget.x += lx;
                        posTarget.y += ly;
                        param.hPos = param.cPos;
                        param.timestamp.hPos = getTimeUsec();
                        cout<<"HomeBase-Detected!"<<endl;
                    }

                    if(tHoverStarted != -1 && getTimeUsec() - tHoverStarted > 2e6)
                    {
                        taskCompleted=true;
                        break; //Why ?!!!
                    }
                }
            }
        }
    }

public:
    Takeoff(QuadParam &q,mavlink_set_position_target_local_ned_t &posT,
            float h, float y, float t):Behaviour(q,posT)
    {
        cout<<"\nTakeoff started to "<<h<<" mts and a yaw of "<<y<<" radians."<<std::flush;
        name="Takeoff";
        targetZ=h;
        targetY=y;
        targetTol=t;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&Takeoff::ruleBase,this);
        }
    }
    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~Takeoff()
    {
        cout<<"\nTakeoff ended and current height is "<<param.cPos.z<<" set initial height to "<<param.iPos.z<<std::flush;
        stop();
    }

};


class Sleep:public Behaviour
{
    void ruleBase()
    {

        posTarget.z=param.cPos.z;
        posTarget.x=param.cPos.x;
        posTarget.y=param.cPos.y;
        posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;

        while(threadRunning && !stopFlag)
        {
            cout<<"\nSleeping"<<std::flush;
            sleep(1);
        }
    }

public:
    Sleep(QuadParam &q, mavlink_set_position_target_local_ned_t &posT):Behaviour(q,posT)
    {
        cout<<"\nSleep started."<<std::flush;
        name="Sleep";
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&Sleep::ruleBase,this);
        }
    }

    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~Sleep()
    {
        cout<<"\nSleep stopped."<<std::flush;
        stop();
    }
};

class MoveInBodyFrame:public Behaviour
{
    float lz,bx,by;
    float targetTol[3];

    float targetY;
    float tolY;
    float yawR;

    mavlink_set_position_target_local_ned_t objPosTarget;

    void ruleBase()
    {
        float lx,ly;
        bodyToNED(bx,by,param.cAtt.yaw,lx,ly);

        posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;
        posTarget.y=param.cPos.y+ly;
        posTarget.x=param.cPos.x+lx;
        posTarget.z=lz;

        int task = 1;

        while(threadRunning)
        {
            if (task ==1)
            {
                while(abs(param.cPos.x-posTarget.x) >= targetTol[0] || \
                        abs(param.cPos.y-posTarget.y) >= targetTol[1] || \
                        abs(param.cPos.z-posTarget.z) >= targetTol[2] && \
                        !stopFlag)
                {
                    //printf("\nDiff: %f %f %f",abs(param.cPos.x-posTarget.x),abs(param.cPos.y-posTarget.y),abs(param.cPos.z-posTarget.z));
                    usleep(1e5);
                }

                //cout<<"\nSuccessfully reached the position within provided tolerance";
                task = 2;
            }

            if (task ==2)
            {
                cout<<"\nYawing to initial angle"<<endl;
                objPosTarget=posTarget;
                Yaw obj(param,objPosTarget,targetY, tolY, yawR);
                obj.start();
                while(!obj.completed() && !stopFlag)
                {
                    posTarget=objPosTarget;
                    usleep(1e5);
                }
                obj.stop();

                taskCompleted=true;
                task = 3;
                break;
            }
        }
    }

public:
    MoveInBodyFrame(QuadParam &q,mavlink_set_position_target_local_ned_t &posT,
                    float x, float y, float z, float tx, float ty, float tz, float yaw, float tYaw, float rate):Behaviour(q,posT)
    {
        cout<<"\nStarted move in body frame for ("<<x<<","<<y<<","<<posTarget.z<<") mts."<<std::flush;
        name="MoveInBodyFrame";
        bx=x;
        by=y;
        lz=z;

        targetTol[0]=tx;
        targetTol[1]=ty;
        targetTol[2]=tz;

        targetY = yaw;
        tolY = tYaw;
        yawR = rate;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&MoveInBodyFrame::ruleBase,this);
        }
    }
    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~MoveInBodyFrame()
    {
        printf("\nMove in body frame (%f,%f,%f,%f) ended.",bx,by,lz,targetY);
        stop();
    }

};


class AvoidBoundary:public Behaviour
{

    float targetDist=0.25;
    float targetTol=0.05;

    void ruleBase()
    {

        Behaviour *obj;
        /*
                if(param.imgData.boundary.dir==1)
                    obj=new MoveInX(param,posTarget,1.*targetDist,targetTol);
                if(param.imgData.boundary.dir==2)
                    obj=new MoveInY(param,posTarget,-1.*targetDist,targetTol);
                if(param.imgData.boundary.dir==3)
                    obj=new MoveInX(param,posTarget,-1.*targetDist,targetTol);
                if(param.imgData.boundary.dir==4)
                    obj=new MoveInY(param,posTarget,1.*targetDist,targetTol);
        */
        while(threadRunning)
        {
            if(!taskCompleted)
            {
                obj->start();
                if(obj->completed())
                {
                    obj->stop();
                    taskCompleted=true;
                }
            }
            usleep(50000);
        }
        obj->stop();
        delete obj;
    }

public:
    AvoidBoundary(QuadParam &q, mavlink_set_position_target_local_ned_t &posT,
                  float t):Behaviour(q,posT)
    {
        cout<<"\nStarted avoid boundary "<<std::flush;
        name="AvoidBoundary";
        targetTol=t;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&AvoidBoundary::ruleBase,this);
        }
    }
    void stop()
    {
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }

        taskCompleted=true;
    }

    ~AvoidBoundary()
    {
        cout<<"\nAvoid boundary stopped."<<std::flush;
        stop();
    }

};



class AvoidObstacle:public Behaviour
{
    void ruleBase()
    {
        int task=1;
        mavlink_set_position_target_local_ned_t objPosTarget;

        while(threadRunning)
        {
            if(!taskCompleted)
            {
                if(task==1)
                {
                    objPosTarget=posTarget;
                    MoveInBodyFrame obj(param,objPosTarget,
                                        0.,-0.7, param.iPos.z, 0.2, 0.3, 0.1,
                                        param.cAtt.yaw, 0.1, 0.1);
                    obj.start();
                    while(!obj.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;
                        usleep(1e5);
                    }
                    obj.stop();

                    if(param.fSonarDist > 2)
                    {
                        usleep(1e5);
                        posTarget.x=param.cPos.x;
                        posTarget.y=param.cPos.y;
                        posTarget.z=param.cPos.z;
                        posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;
                        cout<<"\nDone avoiding obstacle"<<endl;
                        task = 3;
			taskCompleted=true;
			break;
                    }
                }

                if(task==2)
                {
                    //Move 40 feet towards search zone (7m,-0m)
                    cout<<"\nMoving ahead to clear obstacle"<<endl;
                    objPosTarget=posTarget;
                    MoveInBodyFrame obj(param,objPosTarget,
                                        2.5, 0., param.iPos.z, 0.1, 0.05, 0.1,
                                        param.iAtt.yaw, 0.1, 0.1);
                    obj.start();
                    while(!obj.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;
                        usleep(100000);
                    }
                    obj.stop();

                    taskCompleted = true;
                    break;
                }
            }
        }
    }

public:
    AvoidObstacle(QuadParam &q, mavlink_set_position_target_local_ned_t &posT):Behaviour(q,posT)
    {
        cout<<"\nStarted avoiding obstacle"<<std::flush;
        name="AvoidObstacle";
        //targetTol=t;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&AvoidObstacle::ruleBase,this);
        }
    }
    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~AvoidObstacle()
    {
        cout<<"\nAvoiding obstacle stopped."<<std::flush;
        stop();
    }

};



class SearchForTarget:public Behaviour
{
    void ruleBase()
    {
        int task=1;
        bool stop=false;
        mavlink_set_position_target_local_ned_t objPosTarget;

        while(threadRunning)
        {
		//cout<<param.imgData.dropoff.mX<<endl;
            if(!taskCompleted)
            {
                if(task==1 && !stopFlag)
                {
                    //Move 40 feet towards search zone (7m,-0m)
                    cout<<"\nMoving to search area"<<endl;
                    objPosTarget=posTarget;
                    MoveInBodyFrame obj(param,objPosTarget,
                                        5.3, 0., param.iPos.z, 0.05, 0.05, 0.1,
                                        param.iAtt.yaw, 0.1, 0.2);
                    obj.start();

                    while(!obj.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;
                        //cout<<abs(param.cPos.x-lx)<<" "<<abs(param.cPos.y-ly)<<endl;
                        usleep(100000);
                    }
                    obj.stop();

                    param.tPos = param.cPos;
                    param.timestamp.tPos = getTimeUsec();

                    task = 2;
                }

                if(task==2 && !stopFlag)
                {
                    //Search pattern across area
                    //cout<<"\nSearching for target "<<param.imgData.dropoff.mX<<endl;
                    objPosTarget=posTarget;
                    MoveInBodyFrame obj2(param,objPosTarget,
                                        0.5, 0.5, param.iPos.z, 0.1, 0.1, 0.1,
                                        param.iAtt.yaw, 0.1, 0.1);
                    obj2.start();
                    while(!obj2.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;
                        cout << param.imgData.dropoff.mX<<endl;

                        if(param.imgData.dropoff.mX>-1)
                        {
                            cout<<"\nSaw target. Stopping search."<<endl;

                            posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;
                            posTarget.x = param.cPos.x;
                            posTarget.y = param.cPos.y;
                            task=3;

                            break;
                        }
                    }
                    obj2.stop();

                    objPosTarget=posTarget;
                    MoveInBodyFrame obj3(param,objPosTarget,
                                        0., -1., param.iPos.z, 0.05, 0.1, 0.1,
                                        param.iAtt.yaw, 0.1, 0.1);
                    obj3.start();
                    while(!obj3.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;
                        cout << param.imgData.dropoff.mX<<endl;

                        if(param.imgData.dropoff.mX>-1)
                        {
                            cout<<"\nSaw target. Stopping search."<<endl;

                            posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;
                            posTarget.x = param.cPos.x;
                            posTarget.y = param.cPos.y;
                            task=3;

                            break;
                        }
                    }
                    obj3.stop();

                    posTarget.x=param.tPos.x;
                    posTarget.y=param.tPos.y;
                    while(abs(param.cPos.x-posTarget.x) >= 0.1 || \
                        abs(param.cPos.y-posTarget.y) >= 0.1 && \
                        !stopFlag)
                    {
                        if(param.imgData.dropoff.mX>-1)
                        {
                            cout<<"\nSaw target. Stopping search."<<endl;

                            posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;
                            posTarget.x = param.cPos.x;
                            posTarget.y = param.cPos.y;
                            task=3;

                            break;
                        }
                    }

                    task=3;
                }

                if(task==3 && !stopFlag)
                {
                    cout<<"\nHovering for carpet bombing";
                    usleep(5e6);
                    //drop the shiz;
                    taskCompleted=true;
		   param.iAtt.yaw = param.cAtt.yaw;
                    break;
                }

            }
        }
    }

public:
    SearchForTarget(QuadParam &q, mavlink_set_position_target_local_ned_t &posT,
                    float t):Behaviour(q,posT)
    {
        cout<<"\nStarted searching for target "<<std::flush;
        name="SearchForTarget";
        //targetTol=t;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&SearchForTarget::ruleBase,this);
        }
    }
    void stop()
    {
		cout<<"\n--------------------STOPPIN SEARCH"<<endl;
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~SearchForTarget()
    {
        cout<<"\nSearching for target stopped."<<std::flush;
        stop();
    }

};

class SearchForPickup:public Behaviour
{
    void ruleBase()
    {
        int task=1;
        bool stop=false;
        mavlink_set_position_target_local_ned_t objPosTarget;

        while(threadRunning)
        {
            if(!taskCompleted)
            {
                if(task==1 && !stopFlag)
                {
                    //Move 40 feet towards search zone (7m,-0m)
                    cout<<"\nMoving to pickup area"<<endl;
                    objPosTarget=posTarget;
                    MoveInBodyFrame obj(param,objPosTarget,
                                        -1.48, -1.6, param.iPos.z, 0.05, 0.05, 0.1,
                                        param.cAtt.yaw, 0.1, 0.2);
                    obj.start();
                    float lx,ly;
                    bodyToNED(5,0.,param.cAtt.yaw,lx,ly);
                    while(!obj.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;
                        cout<<abs(param.cPos.x-lx)<<" "<<abs(param.cPos.y-ly)<<endl;
                        usleep(100000);
                    }
                    obj.stop();

                    task = 3;

                }

                if(task==2 && !stopFlag)
                {
                    //Search pattern across area
                    cout<<"\nSearching for pickup"<<endl;
                    objPosTarget=posTarget;
                    MoveInBodyFrame obj(param,objPosTarget,
                                        5, 0., param.iPos.z, 0.05, 0.05, 0.1,
                                        param.iAtt.yaw, 0.1, 0.2);
                    obj.start();
                    while(!obj.completed() && !stopFlag)
                    {
                        posTarget=objPosTarget;

                        if(param.imgData.dropoff.pxX>-1)
                        {
                            cout<<"\nSaw pickup. Stopping search."<<endl;

                            posTarget.type_mask=MAVLINK_MSG_SET_POS_TAR_L_NED_POSITION;
                            posTarget.x=param.cPos.x;
                            posTarget.y=param.cPos.y;
                            task=3;


                            break;
                        }
                    }

                    obj.stop();
                    task=3;
                }

                if(task==3 && !stopFlag)
                {
		    posTarget.z=-0.3;
                    cout<<"\nHovering for extraction";
                    usleep(5e6);
                    //drop the shiz;
                    taskCompleted=true;
                    break;
                }

            }
        }
    }

public:
    SearchForPickup(QuadParam &q, mavlink_set_position_target_local_ned_t &posT,
                    float t):Behaviour(q,posT)
    {
        cout<<"\nStarted searching for pickup "<<std::flush;
        name="SearchForPickup";
        //targetTol=t;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&SearchForPickup::ruleBase,this);
        }
    }

    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~SearchForPickup()
    {
        cout<<"\nSearching for pickup stopped."<<std::flush;
        stop();
    }

};


class SearchForHomebase:public Behaviour
{
    void ruleBase()
    {
        int task=1;
        bool stop=false;
        mavlink_set_position_target_local_ned_t objPosTarget;

        while(threadRunning)
        {
            if(!taskCompleted)
            {
                if(task==1 && !stopFlag)
                {
                    //Move 40 feet towards search zone (7m,-0m)
                    cout<<"\nMoving to home base area"<<endl;
                    posTarget.x=param.iPos.x;
                    posTarget.y=param.iPos.y;
                    posTarget.z=param.iPos.z;

                    while(abs(param.cPos.x-posTarget.x) >= 0.1 || \
                            abs(param.cPos.y-posTarget.y) >= 0.1 && \
                            abs(param.cPos.z-posTarget.z) >= 0.1 && \
                            !stopFlag)
                    {
                        //printf("\nDiff: %f %f %f",abs(param.cPos.x-posTarget.x),abs(param.cPos.y-posTarget.y),abs(param.cPos.z-posTarget.z));
                        usleep(1e5);
                    }

                    cout<<"\nSuccessfully reached the position within provided tolerance";
                    taskCompleted=true;
                    break;
                }

            }
        }
    }

public:
    SearchForHomebase(QuadParam &q, mavlink_set_position_target_local_ned_t &posT,
                      float t):Behaviour(q,posT)
    {
        cout<<"\nStarted searching for homebase"<<std::flush;
        name="SearchForHomebase";
        //targetTol=t;
    }

    void start()
    {
        if(!threadRunning)
        {
            threadRunning=true;
            thread=std::thread(&SearchForHomebase::ruleBase,this);
        }
    }
    void stop()
    {
        stopFlag=true;
        if(threadRunning)
        {
            threadRunning=false;
            if(thread.joinable())
                thread.detach();
        }
    }

    ~SearchForHomebase()
    {
        cout<<"\nSearching for homebase stopped."<<std::flush;
        stop();
    }

};



//------------------------------------------
//-----ACTION SELECTION MECHANISM-----------
//------------------------------------------


//Rule base that governs which behaviour is set
void Action::setBehaviour()
{

    //cout<<param.fSonarDist<<endl;

    if(behaviour.size()!=posVec.size())
        cout<<"\nWARNING: PosVec size not equal to behaviour.";


    //cout<<"\nCurrent behaviours: ";
    //for(unsigned int i=0;i<behaviour.size();i++){
    //	cout<<behaviour[i]->getName()<<" "<<std::flush;
    //}
    //cout<<"\nState: "<<state<<endl;


    //If current behaviour is completed then remove it
    if(behaviour.size())
    {
        if( behaviour[behaviour.size()-1]->completed() )
        {

            //If the behaviour is done, then change state to appropriate one

            //If take off is complete then start searching
            if (behaviour[behaviour.size()-1]->getName()== "Takeoff")
            {
                state=SEARCHFORTARGET;
            }

            //If searching is complete then land
            if (behaviour[behaviour.size()-1]->getName()== "SearchForTarget")
            {
                //Drop envelope
                arduino.sendChar('1');
                arduino.sendChar('1');
                usleep(1e5);
                state=SEARCHFORPICKUP;
            }

            //If searching is complete then land
            if (behaviour[behaviour.size()-1]->getName()== "SearchForPickup")
            {
                //Drop envelope
                arduino.sendChar('5');
                arduino.sendChar('5');
                usleep(1e5);
                state=SEARCHFORHOMEBASE;
            }

            //If searching is complete then land
            if (behaviour[behaviour.size()-1]->getName()== "SearchForHomebase")
            {
                state=LAND;
            }

            /*
            //if searching is done then start landing
            if (behaviour[behaviour.size()-1]->getName()== "moveInSquare"){
            	state=LAND;
            }
            */

            //If done avoiding object then restore previous state
            //and restart the previous behaviour
            if (behaviour[behaviour.size()-1]->getName()== "AvoidObstacle")
            {
                state=pState;
            }

            //Stop the completed behaviour and remove it from the stack
            behaviour[behaviour.size()-1]->stop();
            delete behaviour[behaviour.size()-1];
            behaviour.pop_back();
            delete posVec[posVec.size()-1];
            posVec.pop_back();
        }

    }


    //If no behaviour then set to sleep
    if(behaviour.size()==0)
    {
        posVec.push_back(new mavlink_set_position_target_local_ned_t);
        behaviour.push_back( new Sleep(param,*posVec[0]) );
        behaviour[0]->start();
    }


    //For convenience in typing set goal as last behaviour - LIFO
    Behaviour * goal = behaviour[behaviour.size()-1];


    //If current goal is to sleep then clear out other behaviours
    if(goal->getName()== "Sleep")
    {
        //Free the pointer and then remove the other behaviours
        //until only the behaviour 'sleep' is left

        vector<Behaviour*>::iterator it;

        for(it=behaviour.begin(); it<behaviour.end()-1; it++)
        {
            (*it)->stop();
            delete *it;
            behaviour.erase(it);
        }

        mavlink_set_position_target_local_ned_t* tmp = posVec[posVec.size()-1];
        posVec.clear();
        posVec.push_back(tmp);
    }


    if(state==SLEEP)
    {
        //cout<<"\n"<<param.timestamp.iPos;
        if(param.iAtt.yaw==-7)
            state=TAKEOFF;
        else if(behaviour.size()==1)
            behaviour[0]->start();
        //param.iAtt.yaw=2;
    }

    if(state==TAKEOFF && goal->getName()!="Takeoff")
    {
        image.toggleDetection(0,1,0,0);
        //If only behaviour is sleep, then add behaviour
        //takeoff to 1.5m and yaw to 3.14 with tol of 0.1m
        float initYaw=param.cAtt.yaw;
        posVec.push_back(new mavlink_set_position_target_local_ned_t);
        behaviour.push_back(new Takeoff(param,*posVec[posVec.size()-1],
                                        -1.6, initYaw,.10));
        goal->stop(); //Can comment this to cause confusion???
        goal=behaviour[behaviour.size()-1];
        goal->start();

    }


    if(goal->getName()!= "SearchForTarget" && state==SEARCHFORTARGET)
    {
        image.toggleDetection(0,0,1,0);
        //If and state is search then takeoff has completed
        //start square mission
        posVec.push_back(new mavlink_set_position_target_local_ned_t);
        behaviour.push_back(new SearchForTarget(param,*posVec[posVec.size()-1],
                                                0.1));

        goal->stop(); //Can comment this to cause confusion???
        goal=behaviour[behaviour.size()-1];
        goal->start();

    }

    if(goal->getName()!= "SearchForPickup" && state==SEARCHFORPICKUP)
    {

        //If and state is search then takeoff has completed
        //start square mission
        posVec.push_back(new mavlink_set_position_target_local_ned_t);
        behaviour.push_back(new SearchForPickup(param,*posVec[posVec.size()-1],
                                                0.1));

        goal->stop(); //Can comment this to cause confusion???
        goal=behaviour[behaviour.size()-1];
        goal->start();

    }


    if(goal->getName()!= "SearchForHomebase" && state==SEARCHFORHOMEBASE)
    {

        //If and state is search then takeoff has completed
        //start square mission
        posVec.push_back(new mavlink_set_position_target_local_ned_t);
        behaviour.push_back(new SearchForHomebase(param,*posVec[posVec.size()-1],
                            0.1));

        goal->stop(); //Can comment this to cause confusion???
        goal=behaviour[behaviour.size()-1];
        goal->start();

    }

    if(goal->getName()!= "Land" && state==LAND)
    {
        posVec.push_back(new mavlink_set_position_target_local_ned_t);
        behaviour.push_back(new Land(param,*posVec[posVec.size()-1]));

        goal->stop(); //Can comment this to cause confusion???
        goal=behaviour[behaviour.size()-1];
        goal->start();
    }


    //Avoid obstacle if found if not taking off or landing
    if(param.fSonarDist < 2 && state!=TAKEOFF && state!=LAND
            && goal->getName()!="AvoidObstacle")
    {
	cout<<"\n SONAR: "<<param.fSonarDist;
        pState=state;
        state=AVOIDOBSTACLE;
        posVec.push_back(new mavlink_set_position_target_local_ned_t);
        behaviour.push_back(new AvoidObstacle(param,*posVec[posVec.size()-1]));

        //goal->stop(); //Can comment this to cause confusion???
        goal=behaviour[behaviour.size()-1];
        goal->start();
    }


    cPosTarget=*posVec[posVec.size()-1];
}


//Event handler. Main function goes here and it decides what
//action to take.
void Action::eventHandler()
{

    uint64_t sTime,iterTime;

    uint64_t msgTimestamp;
    msgTimestamp=getTimeUsec();

    //image.se(1,0,1,0);

    float comm[8];

    while(threadRunning)
    {
        sTime=getTimeUsec();

        /*
        if(buffers.GSComm.size()){
        	if(buffers.GSComm.pop(comm,8)){
        		switch((int)comm[0]){
        			case 1:
        				cout<<"\nArm command: "<<comm[1]<<std::flush;
        				if(comm[1]==1){

        					param.iPosTarget.yaw=7;
        					param.iPosTarget.z=0;
        				}

        				mavlink_msg_command_long_pack(sysID,compID,&msg,1,0,400,1,comm[1],0,0,0,0,0,0);

        				usleep(1000000);
        			break;

        			case 2:
        				cout<<"\nOffboard command: "<<comm[1]<<std::flush;
        				mavlink_msg_command_long_pack(sysID,compID,&msg,1,0,92,1,comm[1],0,0,0,0,0,0);
        				pixhawk.sendMessage(msg);
        				if(comm[1]==1)
        					obFlag=true;
        				if(comm[1]==0)
        					obFlag=false;

        				usleep(1000000);
        			break;
        		}
        	}
        }
        */



        //update parameters of the quad
        getParameters();
        //cout<<"\nTarget: "<<param.imgData.target.pxX;

        //Sending setpoints at atleast 4 Hz - this will not be exact but is fine now
        if(sTime-msgTimestamp>200000)
        {

            //Debug purposes to see local position in XYZ frame relative to 0,0,0
            //float x,y;
            //NEDTobody(param.cPos.x,param.cPos.y,param.cAtt.yaw,x,y);
            // printf("\nPosition is %f,%f,%f",x,y,param.cPos.z);

            //cout<<param.imgData.target.pxX<<endl;
            //cout<<"\nSending setpoint."<<(int)(sTime-msgTimestamp)<<std::endl;
            mavlink_msg_set_position_target_local_ned_encode(sysID,compID,&msg,&cPosTarget);
            pixhawk.sendMessage(msg);
            //cout<<endl;
            //printf("\n%f %f %f %f %f %f %d %f %f %f %f",\
            cPosTarget.x,cPosTarget.y,cPosTarget.z,cPosTarget.vx,cPosTarget.vy,cPosTarget.vz,cPosTarget.type_mask, param.cPos.x, param.cPos.y, param.cAtt.yaw, param.imgData.target.pxX);
            msgTimestamp=sTime;

            //cout<<"\nSonar: "<<param.fSonarDist;
            //if(param.fSonarDist<500)
            //  arduino.sendChar('1');

        }

        //WORKS CAUSE OUR CONTROLLER IS ALIVE :) :')

        //If offboard then set behaviour
        //if in offboard mode then start going mental. Each hit = 100 pts, partial injury=50 pts
        //cout<<"\nCurrent mode: "<<buffers.messages.heartbeat.custom_mode<<endl;

        //Check if offboard mode is selected
        if(buffers.messages.heartbeat.custom_mode==MODE_OFFBOARD && obFlag==false)
        {
            cout<<"\nStarted offboard mode."<<std::flush;
            obFlag=true;
        }

        //Check if stabilized mode is selected
        if(buffers.messages.heartbeat.custom_mode==MODE_STABILIZED && obFlag==true)
        {
            cout<<"\nStarted stabilized mode."<<std::flush;
            obFlag=false;

            for(size_t i=0; i<behaviour.size(); i++)
            {
                behaviour[i]->stop();
                delete behaviour[i];
                delete posVec[i];
            }

            behaviour.clear();
            posVec.clear();
        }

        //If offboard start
        if(obFlag)
        {
            setBehaviour();
        }


        //Slowing down loop to keep constant ~200000 to reduce CPU usage
        iterTime=getTimeUsec()-sTime;
        if(iterTime<25000)
            usleep(25000-iterTime);
        //cout<<"\nLoop time: "<<(unsigned int)(getTimeUsec()-sTime);
    }

}


void Action::start()
{
    if(!threadRunning)
    {
        threadRunning=true;
        thread=std::thread(&Action::eventHandler,this);
        cout<<"\nStarted event handler"<<std::flush;
    }
}

void Action::stop()
{
    if(threadRunning)
    {
        for(size_t i=0; i<behaviour.size(); i++)
        {
            behaviour[i]->stop();
            delete behaviour[i];
            delete posVec[i];
        }

        behaviour.clear();
        posVec.clear();

        threadRunning=false;
        if(thread.joinable())
        {
            thread.join();
            cout<<"\nStopping event handler"<<std::flush;
        }
    }
}

