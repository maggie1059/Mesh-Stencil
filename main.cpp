#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>

#include "mesh.h"


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

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() != 2) {
        std::cerr << "Error: Wrong number of arguments" << std::endl;
        a.exit(1);
        return 1;
    }
    QString infile = args[0];
    QString outfile = args[1];
    ////////////////////////////////////////////////////////////////////////////////

    Mesh m;
    m.loadFromFile(infile.toStdString());

    // Here's where you'll do all the heavy-lifting: convert the Mesh m into your
    //    own mesh data structure for efficient geometry processing, apply some
    //    operation to that data structure, then convert it back to a basic Mesh.

    m.saveToFile(outfile.toStdString());

    a.exit();
}
