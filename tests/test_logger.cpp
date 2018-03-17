#include <gtest/gtest.h>
#include "../src/logger.h"
#include <fstream>
#include <stdio.h>
using namespace std;

TEST(Logger, log) {
    Logger log;
    char filepath[10] = "./log.csv";

    ifstream Infield(filepath);
    ASSERT_TRUE( Infield.good() );

    remove(filepath);

    fstream Infield2(filepath);
    ASSERT_TRUE( !Infield2.good() );
}