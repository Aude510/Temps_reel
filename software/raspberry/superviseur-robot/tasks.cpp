/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

Tasks::Tasks() : camera(sm, 5) {}

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    CreateMutex(&mutex_monitor);
    CreateMutex(&mutex_robot);
    CreateMutex(&mutex_robotStarted);
    CreateMutex(&mutex_move);
    CreateMutex(&mutex_cameraOpen);
    CreateMutex(&mutex_drawArena);
    CreateMutex(&mutex_drawPosition); 
    CreateMutex(&mutex_conterErrorCom);

    cout << "Mutexes created successfully" << endl << flush;

    CreateSemaphore(&sem_barrier); 
    CreateSemaphore(&sem_openComRobot); 
    CreateSemaphore(&sem_serverOk); 
    CreateSemaphore(&sem_startRobot); 
    CreateSemaphore(&sem_battery); 
    CreateSemaphore(&sem_camera); 
    CreateSemaphore(&sem_camera_images); 
    CreateSemaphore(&sem_camera_close);
    CreateSemaphore(&sem_camera_ask_arena);

    cout << "Semaphores created successfully" << endl << flush;

    CreateTask(&th_server, PRIORITY_TSERVER, "th_server");
    CreateTask(&th_sendToMon, PRIORITY_TSENDTOMON, "th_sendToMon");
    CreateTask(&th_receiveFromMon, PRIORITY_TRECEIVEFROMMON, "th_receiveFromMon");
    CreateTask(&th_openComRobot, PRIORITY_TOPENCOMROBOT, "th_openComRobot");
    CreateTask(&th_startRobot, PRIORITY_TSTARTROBOT, "th_startRobot");
    CreateTask(&th_move, PRIORITY_TMOVE, "th_move");
    CreateTask(&th_batteryLevel, PRIORITY_TSENDTOMON, "th_batteryLevel");
    CreateTask(&th_startCamera, PRIORITY_TCAMERA, "th_startCamera");
    CreateTask(&th_cameraImages, PRIORITY_TCAMERA, "th_cameraImages");
    CreateTask(&th_cameraClose, PRIORITY_TCAMERA, "th_cameraClose");
    CreateTask(&th_askArena, PRIORITY_TCAMERA, "th_askArena");

    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/

    int err;

    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Queues created successfully" << endl << flush;
}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);

    StartTask(&th_server, (void*)&Tasks::ServerTask);
    StartTask(&th_sendToMon, (void*)&Tasks::SendToMonTask);
    StartTask(&th_receiveFromMon, (void*)&Tasks::ReceiveFromMonTask);
    StartTask(&th_openComRobot, (void*)&Tasks::OpenComRobot);
    StartTask(&th_startRobot, (void*)&Tasks::StartRobotTask);
    StartTask(&th_move, (void*)&Tasks::MoveTask);
    StartTask(&th_batteryLevel, (void*)&Tasks::SendBatLevel);
    StartTask(&th_startCamera, (void*)&Tasks::StartCameraTask);
    StartTask(&th_cameraImages, (void*)&Tasks::SendImagesTask); 
    StartTask(&th_cameraClose,(void*)&Tasks::CloseCameraTask); 
    StartTask(&th_askArena,(void*)&Tasks::ConfirmArenaTask); 

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };

    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) 
        {
            delete(msgRcv);
            exit(-1);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) 
        {
            rt_sem_v(&sem_openComRobot);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) 
        {
            rt_sem_v(&sem_startRobot);
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) || msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) || msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) || msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) || msgRcv->CompareID(MESSAGE_ROBOT_STOP)) 
        {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
      
        } 
        else if (msgRcv->CompareID(MESSAGE_ROBOT_BATTERY_GET)) 
        {
            rt_sem_v(&sem_battery);

        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) 
        {
            rt_sem_v(&sem_camera);

        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) 
        {
            rt_sem_v(&sem_camera_close); 
        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA))
        {
            rt_sem_v(&sem_camera_ask_arena); 
        }  
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) 
        {
            rt_mutex_acquire(&mutex_drawArena, TM_INFINITE);
            rt_mutex_acquire(&mutex_cameraOpen, TM_INFINITE);
            drawArena = true;
            camera.Open();
            rt_mutex_release(&mutex_cameraOpen);
            rt_mutex_release(&mutex_drawArena);
            rt_sem_v(&sem_camera_images);
        } 
        else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) 
        {
            rt_mutex_acquire(&mutex_drawArena, TM_INFINITE);
            rt_mutex_acquire(&mutex_cameraOpen, TM_INFINITE);
            drawArena = false;
            camera.Open();
            rt_mutex_release(&mutex_cameraOpen);
            rt_mutex_release(&mutex_drawArena);
            rt_sem_v(&sem_camera_images);
        }
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START))
        {
            rt_mutex_acquire(&mutex_drawPosition, TM_INFINITE); 
            drawPosition = true; 
            rt_mutex_release(&mutex_drawPosition); 
        }
        
        else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP))
        {
            rt_mutex_acquire(&mutex_drawPosition, TM_INFINITE); 
            drawPosition = false; 
            rt_mutex_release(&mutex_drawPosition); 
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message* msgSend;
        if (status < 0) 
        {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } 
        else 
        {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (true) 
    {
        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        rt_mutex_acquire(&mutex_conterErrorCom, TM_INFINITE);
        if (msgSend->GetID() == MESSAGE_ANSWER_COM_ERROR)
        {
            nbErrorCom++;
            if (nbErrorCom>=3)
            {
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_COM_ERROR));
                robot.Close();
                rt_sem_v(&sem_openComRobot);
            }
        }
        else 
        {
            nbErrorCom=0;
            rt_mutex_release(&mutex_conterErrorCom);
            cout << msgSend->GetID();
            cout << ")" << endl;

            cout << "Movement answer: " << msgSend->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            }
        }
        
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000); // en ns?

    while (true) 
    {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            Message * msgSend = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_acquire(&mutex_conterErrorCom, TM_INFINITE);
            if (msgSend->GetID() == MESSAGE_ANSWER_COM_ERROR)
            {
                nbErrorCom++;
                rt_mutex_release(&mutex_conterErrorCom);
                if (nbErrorCom>=3)
                {
                    WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_COM_ERROR));
                    robot.Close();
                    rt_sem_v(&sem_openComRobot);
                }
            }
            else 
            {
                nbErrorCom=0;
                rt_mutex_release(&mutex_conterErrorCom);
                rt_mutex_release(&mutex_robot);
            }
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0)
    {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }

    return msg;
}



// ----------------------------------------------------------------------------------------------

void Tasks::CreateMutex(RT_MUTEX* mutex) 
{
    int err;
    
    if (err = rt_mutex_create(mutex, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
}

void Tasks::CreateSemaphore(RT_SEM* semaphore)
{
    int err;

    if (err = rt_sem_create(semaphore, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
}

void Tasks::CreateTask(RT_TASK* task, int priority, const char* taskName)
{
    int err;

    if (err = rt_task_create(task, taskName, 0, priority, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
}

void Tasks::StartTask(RT_TASK* task, void* callback)
{
    if (int err = rt_task_start(task, (void(*)(void*))callback, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
}

/**
 * @brief Thread handling control of the battery level.
 fonctionnalité 13
 */
void Tasks::SendBatLevel(void *arg) 
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);


    rt_task_set_periodic(NULL, TM_NOW, 500000000); // en ns?

    rt_sem_p(&sem_battery, TM_INFINITE); // pour attendre que la case soit cochée

    while(1)
    {
        rt_task_wait_period(NULL);
        cout << "send battery" << endl << flush;
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        MessageBattery* lvlBat = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET));
        rt_mutex_acquire(&mutex_conterErrorCom, TM_INFINITE);
        if (lvlBat->GetID() == MESSAGE_ANSWER_COM_ERROR){
            nbErrorCom++;
            rt_mutex_release(&mutex_conterErrorCom);
            if (nbErrorCom>=3)
            {
                WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_COM_ERROR));
                robot.Close();
                rt_sem_v(&sem_openComRobot);
            }
        }
        else
        {
            nbErrorCom=0;
            rt_mutex_release(&mutex_conterErrorCom);
            cout << lvlBat->GetLevel() << endl;
            rt_mutex_release(&mutex_robot);    
            WriteInQueue(&q_messageToMon, lvlBat);
        }
    }
}

//camera task : ouvrir la caméra, fonctionnalité 14

void Tasks::StartCameraTask(void *arg)
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startCamera starts here                                                    */
    /**************************************************************************************/
    while (1) {

        rt_sem_p(&sem_camera, TM_INFINITE);
        cout << "Opening Camera :-) " << endl; 
        rt_mutex_acquire(&mutex_cameraOpen, TM_INFINITE);
        if (!camera.Open()) // problem opening camera 
        {
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK)); //:)
        } 
        else // release sem to send images 
        { 
            rt_sem_v(&sem_camera_images); 
        }
        rt_mutex_release(&mutex_cameraOpen);
        
        cout << "camera opened" << endl;

    }
}

//camera task : envoyer les images depuis la caméra, fonctionnalité 15


void Tasks::SendImagesTask(void *arg)
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    rt_task_set_periodic(NULL, TM_NOW, 100000000); // en ns?

    while (true)
    {
        rt_sem_p(&sem_camera_images, TM_INFINITE); 
        rt_mutex_acquire(&mutex_cameraOpen, TM_INFINITE);
        Img frame = camera.Grab(); // récup image 
        if (drawArena)
        {
            frame.DrawArena(arena); // fonctionnalité 17 
            if (drawPosition) 
            { 
                // fonctionnalité 18
                std::list<Position> list = frame.SearchRobot(arena); 
                if (list.empty())
                {
                    Position position;
                    position.center.x = -1;
                    position.center.y = -1;
                    WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, position)); 
                }
                else
                {
                    WriteInQueue(&q_messageToMon, new MessagePosition(MESSAGE_CAM_POSITION, list.front())); 
                    frame.DrawRobot(list.front()); 
                }
            } 
        }
        rt_mutex_release(&mutex_cameraOpen); 
        WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, &frame)); // envoyer image 
        rt_sem_v(&sem_camera_images); 
    }
}

//camera task : fermer la caméra, fonctionnalité 16
void Tasks::CloseCameraTask(void * arg)
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while (1) {

        rt_sem_p(&sem_camera_close, TM_INFINITE);
        cout << "closing Camera :-) " << endl; 
        rt_sem_p(&sem_camera_images, TM_INFINITE); // arrêter d'envoyer des images 
        rt_mutex_acquire(&mutex_cameraOpen, TM_INFINITE);
        camera.Close(); 
        WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_ACK));  // envoyer ack 
        rt_mutex_release(&mutex_cameraOpen);
        cout << "camera closed" << endl;

    }
}

// fonctionnalité 17 : 
void Tasks::ConfirmArenaTask(void * arg)
{
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while(true)
    {
        rt_sem_p(&sem_camera_ask_arena, TM_INFINITE);
        
        cout << "Drawing Arena :-) " << endl; 
        
        rt_sem_p(&sem_camera_images, TM_INFINITE);
        rt_mutex_acquire(&mutex_cameraOpen, TM_INFINITE);
        
        Img frame = camera.Grab(); // récup image
        camera.Close();
        
        rt_mutex_release(&mutex_cameraOpen);

        arena = frame.SearchArena();

        if(!arena.IsEmpty())
        {
            cout << arena.ToString() << endl << flush;
            frame.DrawArena(arena);
            WriteInQueue(&q_messageToMon, new MessageImg(MESSAGE_CAM_IMAGE, &frame));
        }
        else
        {
            cout << "No arena found" << endl << flush;
            WriteInQueue(&q_messageToMon, new Message(MESSAGE_ANSWER_NACK));
        }
    }
}

/*TODO : Fonctionnalité 5-6
Tester 8-9
10-11 Watchdog
12 : ne pas envoyer stop tout le temps
*/