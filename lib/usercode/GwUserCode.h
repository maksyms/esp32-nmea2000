#ifndef _GWUSERCODE_H
#define _GWUSERCODE_H
#include <Arduino.h>
#include <map>
#include "GwApi.h"
#include "GwJsonDocument.h"
class GwLog;

class GwApiInternal : public GwApi{
    public:
    ~GwApiInternal(){}
    virtual void fillStatus(GwJsonDocument &status){};
    virtual int getJsonSize(){return 0;};
    virtual bool handleWebRequest(const String &url,AsyncWebServerRequest *req){return false;}
};
class GwUserTask{
    public:
        String name;
        TaskFunction_t task=NULL;
        GwUserTaskFunction usertask=NULL;
        bool isUserTask=false;
        GwApiInternal *api=NULL;
        int stackSize=2000;
        GwUserTask(String name,TaskFunction_t task,int stackSize=2000){
            this->name=name;
            this->task=task;
            this->stackSize=stackSize;
        }
        GwUserTask(String name, GwUserTaskFunction task,int stackSize=2000){
            this->name=name;
            this->usertask=task;
            this->isUserTask=true;
            this->stackSize=stackSize;
        }
};

class TaskInterfacesStorage;
class GwUserCode{
    GwLog *logger;
    GwApiInternal *api;
    SemaphoreHandle_t *mainLock;
    TaskInterfacesStorage *taskData;
    void startAddOnTask(GwApiInternal *api,GwUserTask *task,int sourceId,String name);
    public:
        ~GwUserCode();
        typedef std::map<String,String> Capabilities;
        GwUserCode(GwApiInternal *api, SemaphoreHandle_t *mainLock);
        void startUserTasks(int baseId);
        void startInitTasks(int baseId);
        void startAddonTask(String name,TaskFunction_t task, int id);
        Capabilities *getCapabilities();
        void fillStatus(GwJsonDocument &status);
        int getJsonSize();
        void handleWebRequest(const String &url,AsyncWebServerRequest *);
};
#endif