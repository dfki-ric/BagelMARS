/**
 * \file BagelMARS.hpp
 * \author Malte Langosz (malte.langosz@dfki.de)
 * \brief
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_BEHAVIOR_GRAPH_MARS_H
#define MARS_PLUGINS_BEHAVIOR_GRAPH_MARS_H

#ifdef _PRINT_HEADER_
  #warning "BagelMARS.h"
#endif

// set define if you want to extend the gui
#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/DataPackageMapping.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/main_gui/MenuInterface.h>

#include <configmaps/ConfigData.h>

#include <cpp_bagel_wrapper/BagelGraph.hpp>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/utils/Mutex.h>

#include <string>

namespace mars {

  namespace plugins {
    namespace BagelMARS {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class BagelMARS: public mars::interfaces::MarsPluginTemplateGUI,
        public mars::data_broker::ReceiverInterface,
        public mars::data_broker::ProducerInterface,
        // for gui
        public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {

      public:
        BagelMARS(lib_manager::LibManager *theManager);
        ~BagelMARS();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("BagelMARS"); }

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,
                                 const data_broker::DataPackage &package,
                                 int callbackParam);
        virtual void produceData(const data_broker::DataInfo &info,
                                 data_broker::DataPackage *dbPackage,
                                 int callbackParam);
        // CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        void menuAction(int action, bool checked = false);

        // BagelMARS methods
        void setParameter(std::vector<std::string> names,
                          std::vector<double> values);

        void update(const std::string &filename, configmaps::ConfigMap map);


      private:
        cfg_manager::cfgPropertyStruct example;
        data_broker::DataPackageMapping dbPackageMapping;

        cpp_bagel_wrapper::BagelGraph *bagelGraph;

        bool gActive, createMotorDBItems;
        std::vector<double> inputValues;
        std::vector<double> outputValues;
        std::map<unsigned long, unsigned long> inputMap;
        std::vector<std::string> inputNames;
        std::vector<std::string> outputNames;
        std::map<unsigned long, unsigned long> motorMap;
        std::map<unsigned long, unsigned long> motorForceMap;
        std::map<unsigned long, unsigned long> motorGraphSimMap;
        std::map<unsigned long, unsigned long> motorTorqueGraphSimMap;
        std::map<std::string, size_t> mapInputValues;

        interfaces::NodeData conveyor;
        unsigned long conveyorPosZId, conveyorSpeedXId;
        std::string graphFilename, externNodesPath, startParameters;
        double timeStep;
        utils::Mutex mutex;
        void reloadGraph();

      }; // end of class definition BagelMARS

    } // end of namespace BagelMARS
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_BEHAVIOR_GRAPH_MARS_H
