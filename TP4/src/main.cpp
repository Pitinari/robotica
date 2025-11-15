#include <iostream>
#include <string>
#include "graph_slam_2d.h"
#include "graph_slam_3d.h"

using namespace std;

void printUsage(const char *programName)
{
    cout << "Usage: " << programName << " <mode> <g2o_file> <output_prefix>" << endl;
    cout << "\nModes:" << endl;
    cout << "  2d-batch       - 2D batch optimization (Gauss-Newton)" << endl;
    cout << "  2d-incremental - 2D incremental optimization (ISAM2)" << endl;
    cout << "  3d-batch       - 3D batch optimization (Gauss-Newton)" << endl;
    cout << "  3d-incremental - 3D incremental optimization (ISAM2)" << endl;
    cout << "\nExample:" << endl;
    cout << "  " << programName << " 2d-batch input_INTEL_g2o.g2o results/2d_batch" << endl;
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        printUsage(argv[0]);
        return 1;
    }

    string mode = argv[1];
    string g2oFile = argv[2];
    string outputPrefix = argv[3];

    try
    {
        if (mode == "2d-batch")
        {
            optimize2DBatch(g2oFile, outputPrefix);
        }
        else if (mode == "2d-incremental")
        {
            optimize2DIncremental(g2oFile, outputPrefix);
        }
        else if (mode == "3d-batch")
        {
            optimize3DBatch(g2oFile, outputPrefix);
        }
        else if (mode == "3d-incremental")
        {
            optimize3DIncremental(g2oFile, outputPrefix);
        }
        else
        {
            cerr << "Error: Unknown mode '" << mode << "'" << endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    catch (const exception &e)
    {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    cout << "\nDone! Results saved to " << outputPrefix << "_*.csv" << endl;
    cout << "Use the Python visualization script to plot the results." << endl;

    return 0;
}
