#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>
#include <chrono>

#include "mesh.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    ////////////////////////////////////////////////////////////////////////////////
    // You will likely need to add more command line arguments here, e.g. to specify
    //    which operation to perform on the input mesh.
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("infile", "Input .obj file path");
    parser.addPositionalArgument("outfile", "Output .obj file path");
    parser.addPositionalArgument("method", "subdivide/simplify/remesh/denoising");
    parser.addPositionalArgument("args1", "respective argument for the method");
    parser.addPositionalArgument("args2", "respective argument2 for the method");
    //TODO: list the options

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() < 4) {
        cerr << "Error: Wrong number of arguments" << endl;
        a.exit(1);
        return 1;
    }
    QString infile = args[0];
    QString outfile = args[1];
    QString method = args[2];

    ////////////////////////////////////////////////////////////////////////////////

    Mesh m;
    m.loadFromFile(infile.toStdString());

    auto t0 = high_resolution_clock::now();
    // TODO
    // Convert the mesh into your own data structure

    // TODO
    // Implement the operations
    if (method == "subdivide"){
        //TODO
    } else if (method == "simplify"){
        //TODO
    } else if (method == "remesh"){
        //TODO
    } else if (method == "denoising") {
        //TODO
    } else {
        cerr << "Error: Unknown method name" << endl;
    }

    auto t1 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t1-t0).count();

    cout << "Execution takes: " << duration << " milliseconds." <<endl;
    // TODO
    // Convert your datastructure back to the basic format

    ////////////////////////////////////////////////////////////////////////////////
    m.saveToFile(outfile.toStdString());

    a.exit();
}
