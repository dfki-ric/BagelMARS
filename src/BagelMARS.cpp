/**
 * \file BagelMARS.cpp
 * \author Malte Langosz (malte.langosz@dfki.de)
 * \brief
 *
 * Version 0.1
 */


#include "BagelMARS.hpp"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/JointManagerInterface.h>
#include <mars/utils/misc.h>

#include <sys/time.h>

namespace mars {
  namespace plugins {
    namespace BagelMARS {

      using namespace configmaps;
      using namespace mars::utils;
      using namespace mars::interfaces;

      BagelMARS::BagelMARS(lib_manager::LibManager *theManager)
        : MarsPluginTemplateGUI(theManager, "BagelMARS"),
          gActive(false) {
      }

      void BagelMARS::init() {
        ConfigMap map;

        if(pathExists("BagelMARS.yml")) {
          map = ConfigMap::fromYamlFile("BagelMARS.yml");
        }
        else if(pathExists("BagelGraphMARS.yml")) {
          map = ConfigMap::fromYamlFile("BagelGraphMARS.yml");
        }
        else {
          return;
        }

        //graphFilename;
        if(map.hasKey("GraphFilename")) {
          graphFilename << map["GraphFilename"];
        }

        if(map.hasKey("externNodesPath")) {
          externNodesPath << map["externNodesPath"];
        }

        startParameters = "";
        if(map.hasKey("StartParameters")) {
          startParameters << map["StartParameters"];
        }
        createMotorDBItems = false;
        if(map.hasKey("CreateMotorDBItems")) {
          createMotorDBItems = map["CreateMotorDBItems"];
        }
        reloadGraph();

        // register as producer
        control->dataBroker->registerTimedProducer(this, "BagelMARS",
                                                   "outputs",
                                                   "mars_sim/simTimer", 0);

        if(gui) {
          gui->addGenericMenuAction("../Bagel/activate", 1, this, 0, "", 0, 1);
          gui->addGenericMenuAction("../Bagel/createMotorDB", 2, this);
          //gui->addGenericMenuAction("../Bagel/createConveyor", 3, this);
          gui->addGenericMenuAction("../Bagel/reload", 4, this);
        }

        control->cfg->loadConfig("cfgBagelMARS.yml");
        example = control->cfg->getOrCreateProperty("BagelMARS", "timeStep", 40., this);
        timeStep = example.dValue;
      }

      void BagelMARS::reset() {
        bagelGraph->reset();
      }

      BagelMARS::~BagelMARS() {
        libManager->releaseLibrary("behavior_library");
      }

      void BagelMARS::reloadGraph() {
        mutex.lock();
        bagelGraph = new cpp_bagel_wrapper::BagelGraph();
        bagelGraph->loadGraph(graphFilename, externNodesPath);

        ConfigMap map;
        try {
          if(startParameters != ""){
            map = ConfigMap::fromYamlFile(startParameters);
          }else{
            map = ConfigMap::fromYamlFile("start_parameters.yml");
          }
        }
        catch(...) {}

        inputNames = bagelGraph->getInputNames();
        outputNames = bagelGraph->getOutputNames();

        inputValues.clear();
        outputValues.clear();
        inputMap.clear();
        control->dataBroker->unregisterSyncReceiver(this, "*", "*");

        for(unsigned int i=0; i<inputNames.size(); ++i) {
          if(map.find(inputNames[i]) != map.end()) {
            inputValues.push_back(map[inputNames[i]]);
          }
          else {
            inputValues.push_back(0.0);
          }
          mapInputValues[inputNames[i]] = i;
        }

        unsigned long id;
        for(size_t i=0; i<inputNames.size(); ++i) {
          if(find(outputNames.begin(), outputNames.end(),
                  inputNames[i]) == outputNames.end()) {
            //fprintf(stderr, "add %s to DataBroker\n", inputNames[i].c_str());
            mars::data_broker::DataPackage dbPackage;
            dbPackage.add(inputNames[i], inputValues[i]);
            id = control->dataBroker->pushData("BagelMARS", inputNames[i], dbPackage,
                                               this, mars::data_broker::DATA_PACKAGE_READ_WRITE_FLAG);
            inputMap[id] = i;
            control->dataBroker->registerSyncReceiver(this, "BagelMARS",
                                                      inputNames[i], id);
          }
        }

        for(size_t i=0; i<outputNames.size(); ++i) {
          outputValues.push_back(0.0);
        }
        for(size_t i=0; i<outputNames.size(); ++i) {
          //fprintf(stderr, "add to db: %s\n", outputNames[i].c_str());
          dbPackageMapping.add(outputNames[i], &(outputValues[i]));
        }

        data_broker::DataPackage dbPackage;
        dbPackageMapping.writePackage(&dbPackage);
        control->dataBroker->pushData("BagelMARS", "outputs",
                                      dbPackage, NULL,
                                      data_broker::DATA_PACKAGE_READ_FLAG);
        mutex.unlock();
      }

      void BagelMARS::setParameter(std::vector<std::string> names,
                                           std::vector<double> values) {
        for(unsigned int i=0; i<names.size(); ++i) {
          for(unsigned int k=0; k<inputNames.size(); ++k) {
            if(inputNames[k] == names[i]) {
              inputValues[k] = values[i];
              break;
            }
          }
        }
      }


      void BagelMARS::update(sReal time_ms) {
        static bool firstUpdate = true;
        if(firstUpdate) {
          firstUpdate = false;
          menuAction(2);
          menuAction(1);
        }
        mutex.lock();
        static double leftTime = 0.0;
        if(gActive) {
          leftTime += time_ms;
          if(leftTime >= timeStep) {
            leftTime -= timeStep;
            bagelGraph->setInputValues(inputValues);
            bagelGraph->evaluate();
            bagelGraph->getOutputValues(&outputValues);
          }
          std::map<unsigned long, unsigned long>::iterator it;
          for(it=motorGraphSimMap.begin(); it!=motorGraphSimMap.end(); ++it) {
            control->motors->setMotorValue(it->first, outputValues[it->second]);
          }
          for(it=motorTorqueGraphSimMap.begin();
              it!=motorTorqueGraphSimMap.end();
              ++it) {
            control->motors->setMaxTorque(it->first, outputValues[it->second]);
          }
          for(unsigned int i=0; i<outputValues.size(); ++i) {
            // we directly map output to inputs if the names are equal
            if(mapInputValues.find(outputNames[i]) != mapInputValues.end()){
              inputValues[mapInputValues[outputNames[i]]] = outputValues[i];
            }
          }
        }
        mutex.unlock();
      }

      void BagelMARS::receiveData(const data_broker::DataInfo& info,
                                        const data_broker::DataPackage& package,
                                        int id) {
        double value;
        package.get(0, &value);
        if(inputMap.find(id) != inputMap.end()) {
          inputValues[inputMap[id]] = value;
        }
        else if(motorMap.find(id) != motorMap.end()) {
          if(gActive) {
            control->motors->setMotorValue(motorMap[id], value);
          }
        }
        else if(motorForceMap.find(id) != motorForceMap.end()) {
          if(gActive) {
            control->motors->setMaxTorque(motorForceMap[id], value);
          }
        }
        else if(id == conveyorPosZId) {
          conveyor.pos.z() = value;
          control->nodes->editNode(&conveyor, EDIT_NODE_POS);
        }
        else if(id == conveyorSpeedXId) {
          conveyor.c_params.motion1 = value;
          control->nodes->editNode(&conveyor, EDIT_NODE_CONTACT);
        }
        // package.get("force1/x", force);
      }

      void BagelMARS::produceData(const data_broker::DataInfo &info,
                                          data_broker::DataPackage *dbPackage,
                                          int callbackParam) {
        dbPackageMapping.writePackage(dbPackage);
      }

      void BagelMARS::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {


        if(_property.paramId == example.paramId) {
          timeStep = example.dValue = _property.dValue;
        }
      }

      void BagelMARS::update(const std::string &filename,
                                     ConfigMap map) {
        if(filename == graphFilename) {
          reloadGraph();
        }
        else {
          bagelGraph->updateSubGraph(filename, map);
        }
      }

      void BagelMARS::menuAction(int action, bool checked) {
        if(action == 1) {
          gActive = !gActive;
        }
        else if(action == 2) {
          std::vector<interfaces::core_objects_exchange> allMotors;
          std::vector<interfaces::core_objects_exchange>::iterator it;
          unsigned long id;

          motorGraphSimMap.clear();
          motorTorqueGraphSimMap.clear();
          control->motors->getListMotors(&allMotors);
          for(it=allMotors.begin(); it!=allMotors.end(); ++it) {
            if(motorMap.find(it->index) == motorMap.end()) {
              if(createMotorDBItems) {
                mars::data_broker::DataPackage dbPackage;
                std::string name = "motors/";
                name.append(it->name);
                dbPackage.add(name, 0.0);
                id = control->dataBroker->pushData("BagelMARS", name,
                                                   dbPackage,
                                                   this, mars::data_broker::DATA_PACKAGE_READ_WRITE_FLAG);
                control->dataBroker->registerTimedReceiver(this,
                                                           "BagelMARS",
                                                           name,
                                                           "mars_sim/simTimer",
                                                           10, id);
                motorMap[id] = it->index;

                mars::data_broker::DataPackage dbPackage2;
                name = "motors/maxTorques/";
                name.append(it->name);
                dbPackage2.add(name, 0.0);
                id = control->dataBroker->pushData("BagelMARS", name,
                                                   dbPackage2,
                                                   this, mars::data_broker::DATA_PACKAGE_READ_WRITE_FLAG);
                
                control->dataBroker->registerSyncReceiver(this,
                                                          "BagelMARS",
                                                          name, id);
                motorForceMap[id] = it->index;
              }

              std::string cmpName1 = it->name;
              std::string cmpName2 = it->name + "/des_pos";
              std::string cmpName3 = it->name + "/des_angle";
              //search in the graph outputs
              for(int i=0; i<outputNames.size(); ++i) {
                if(outputNames[i] == cmpName1 ||
                   outputNames[i] == cmpName2 ||
                   outputNames[i] == cmpName3) {
                  motorGraphSimMap[it->index] = i;
                fprintf(stderr, "compare: %s - %s\n", outputNames[i].c_str(), cmpName1.c_str());
                }
                std::string name2 = "torque;";
                name2.append(it->name);
                if(outputNames[i] == name2) {
                  motorTorqueGraphSimMap[it->index] = i;
                }
              }
            }
          }

          ConfigMap map;
          try {
            map = ConfigMap::fromYamlFile("db_bg_connection.yml",
                                          true);
          }
          catch(...) {}
          ConfigVector::iterator mapIt;
          if(map.find("Connections") != map.end()) {
            for(mapIt=map["Connections"].begin();
                mapIt!=map["Connections"].end(); ++mapIt) {
              std::string fromGroup, fromData, fromItem;
              std::string toGroup, toData, toItem;

              std::string type = (*mapIt)["type"];
              if(type == "Node") {
                std::string nodeName = (*mapIt)["nodeName"];
                control->nodes->getDataBrokerNames(control->nodes->getID(nodeName),
                                                   &fromGroup, &fromData);
              } else if(type == "Joint") {
                std::string jointName = (*mapIt)["jointName"];
                control->joints->getDataBrokerNames(control->joints->getID(jointName),
                                                   &fromGroup, &fromData);
              } else if(type == "Motor") {
                std::string motorName = (*mapIt)["motorName"];
                control->motors->getDataBrokerNames(control->motors->getID(motorName),
                                                    &fromGroup, &fromData);
              } else {
                fromGroup << (*mapIt)["fromGroup"];
                fromData << (*mapIt)["fromData"];
              }
              fromItem << (*mapIt)["fromItem"];
              toGroup << (*mapIt)["toGroup"];
              toData << (*mapIt)["toData"];
              toItem << (*mapIt)["toItem"];

              fprintf(stderr, "connect: %s %s %s %s %s %s\n", fromGroup.c_str(),
                      fromData.c_str(), fromItem.c_str(), toGroup.c_str(),
                      toData.c_str(), toItem.c_str());

              control->dataBroker->connectDataItems(fromGroup, fromData,
                                                    fromItem,
                                                    toGroup, toData,
                                                    toItem);
            }
          }
        }
        else if(action == 3) {
          // add conveyor
          conveyor.name = "conveyor";
          conveyor.physicMode = NODE_TYPE_PLANE;
          conveyor.ext = Vector(6,6,1);
          conveyor.pos = Vector(0.0, 0.0, 0.0);
          conveyor.origName = "plane";
          conveyor.filename = "PRIMITIVE";
          conveyor.material.diffuseFront = Color(0.4, 0.4, 0.4, 1.0);
          conveyor.c_params.cfm = 0.002;
          conveyor.c_params.friction_direction1 = new Vector(-1.0, 0.0, 0.0);
          conveyor.c_params.motion1 = 0.2;
          control->nodes->addNode(&conveyor);

          mars::data_broker::DataPackage dbPackage1;
          dbPackage1.add("conveyor_pos_z", 0.0);
          conveyorPosZId = control->dataBroker->pushData("BagelMARS", "conveyor_pos_z", dbPackage1,
                                                         this, mars::data_broker::DATA_PACKAGE_READ_WRITE_FLAG);

          control->dataBroker->registerSyncReceiver(this, "BagelMARS",
                                                     "conveyor_pos_z", conveyorPosZId);

          mars::data_broker::DataPackage dbPackage2;
          dbPackage2.add("conveyor_speed_x", 0.2);
          conveyorSpeedXId = control->dataBroker->pushData("BagelMARS", "conveyor_speed_x", dbPackage2,
                                                           this, mars::data_broker::DATA_PACKAGE_READ_WRITE_FLAG);

          control->dataBroker->registerSyncReceiver(this, "BagelMARS",
                                                     "conveyor_speed_x", conveyorSpeedXId);

        }
        else if(action == 4) {
          reloadGraph();
        }
      }

    } // end of namespace BagelMARS
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::BagelMARS::BagelMARS);
CREATE_LIB(mars::plugins::BagelMARS::BagelMARS);
