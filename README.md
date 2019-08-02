# BagelMARS

Bagel (Biologically inspired Graph-Based Language) is a cross-platform
graph-based dataflow language developed at the
[Robotics Innovation Center of the German Research Center for Artificial Intelligence (DFKI-RIC)](http://robotik.dfki-bremen.de/en/startpage.html)
and the [University of Bremen](http://www.informatik.uni-bremen.de/robotik/index_en.php).
It runs on (Ubuntu) Linux, Mac and Windows.

BagelMARS is a plugin for the MARS simulation
(https://github.com/rock-simulation/mars) that can be used to load Bagel
graphs into MARS. The in- and outputs of the loaded graph are connected
to DataBroker.

# General {#general}

The main user documentation of Bagel can be found at:
https://github.com/dfki-ric/bagel_wiki/wiki

The API documentation of `osg_graph_viz` can be build in the `doc`
sub-folder with the `make` command. The documentation is build into
the `doc/build` folder.

## Installation

It can be installed by the build systems (autoproj or pybob) used by the
MARS simulation.

# Test

To test if everything is installed correctly simply start the MARS
simulation in the test folder.

    cd test
    mars_app

The BagelMARS plugin was loaded correctly if the output includes the following
lines:

    BagelGraph::loadGraph: ./ / bagel_test_graph.yml
    [loadGraph] #inputs: 1
    [loadGraph] #outputs: 1


# License

BagelMARS is distributed under the
[3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).
