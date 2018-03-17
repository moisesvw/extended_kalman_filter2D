#include <gtest/gtest.h>
#include "../src/logger.h"
#include <fstream>
#include <stdio.h>
#include <string>
using namespace std;

TEST(Logger, log) {
    Logger log;
    char filepath[10] = "./log.csv";

    ifstream infield(filepath);
    ASSERT_TRUE( infield.good() );

    remove(filepath);

    fstream infield2(filepath);
    ASSERT_TRUE( !infield2.good() );
}

TEST(Logger, logline) {
    char filepath[10] = "./log.csv";
    Logger log;
    log.log("head, head3");
    log.log("c, cc3");
    log.close();

    ifstream infield;
    infield.open(filepath);
    string line1;
    string line2;
    getline(infield, line1);
    getline(infield, line2);
    ASSERT_TRUE( line1 == "head, head3");
    ASSERT_TRUE( line2 == "c, cc3");
    remove(filepath);
}