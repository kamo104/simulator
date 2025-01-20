#pragma once
#include "common.h"
#include "convertions.h"
#include "flightPlan.h"

// This should be a file


const FlightSegment touchdown10 = {
   geo2xy({{52.423969, 16.81117,  0}}), {0.0, hdg2rad(100) }, false, true
};

const FlightSegment touchdown28 = {
   geo2xy({{52.418934, 16.837273, 0}}), {0.0, hdg2rad(280) }, false, true 
};

const FlightSegment exitInner28_1 = {
   geo2xy({{52.422879, 16.816818, 0}})
};

const FlightSegment exitInner28_2 = {
   geo2xy({{52.422952, 16.813683, 0}})
};

const FlightSegment exitInner28_3 = {
   geo2xy({{52.422719, 16.811930, 0}})
};

const FlightSegment exitInner28_4 = {
   geo2xy({{52.422180, 16.811697, 0}})
};

const FlightSegment exitOuter28_1 = {
   geo2xy({{52.424803, 16.806835, 0}})
};

const FlightSegment exitOuter28_2 = {
   geo2xy({{52.424527, 16.805982, 0}})
};

const FlightSegment exitOuter28_3 = {
   geo2xy({{52.423735, 16.805568, 0}})
};

const FlightSegment exitOuter28_4 = {
   geo2xy({{52.423271, 16.805945, 0}})
};

// L10
const FlightSegment exitInner10_1 = {
   geo2xy({{52.419486, 16.834403, 0}})
};

const FlightSegment exitInner10_2 = {
   geo2xy({{52.418775, 16.835902, 0}})
};

const FlightSegment exitInner10_3 = {
   geo2xy({{52.417852, 16.837356, 0}})
};

const FlightSegment exitInner10_4 = {
   geo2xy({{52.417282, 16.837199, 0}})
};

const FlightSegment exitOuter10_1 = {
   geo2xy({{52.418352, 16.840292, 0}})
};

const FlightSegment exitOuter10_2 = {
   geo2xy({{52.417834, 16.840721, 0}})
};

const FlightSegment exitOuter10_3 = {
   geo2xy({{52.416951, 16.840188, 0}})
};

const FlightSegment exitOuter10_4 = {
   geo2xy({{52.416771, 16.839588, 0}})
};

const FlightSegment taxiA = {
   geo2xy({{52.418835, 16.828834, 0}})
};

const FlightSegment taxiB = {
   geo2xy({{52.418479, 16.829166, 0}})
};

const FlightSegment taxiC = {
   geo2xy({{52.416967, 16.828385, 0}})
};

const FlightSegment taxiD = {
   geo2xy({{52.418609, 16.830002, 0}})
};


const FlightSegment parkA = {
   geo2xy({{52.416096, 16.828321, 0}})
};

const int EXIT_START = 1;
const int EXIT_SIZE = 4;
const int LANDING_SIZE = 13;
const std::array <FlightSegment, 13> landing10 = {
    touchdown10,
    exitInner10_1,
    exitInner10_2,
    exitInner10_3,
    exitInner10_4,
    exitOuter10_1,
    exitOuter10_2,
    exitOuter10_3,
    exitOuter10_4,
    taxiD,
    taxiB,
    taxiC,
    parkA
};

const std::array <FlightSegment, 13> landing28 = {
    touchdown28,
    exitInner28_1,
    exitInner28_2,
    exitInner28_3,
    exitInner28_4,
    exitOuter28_1,
    exitOuter28_2,
    exitOuter28_3,
    exitOuter28_4,
    taxiA,
    taxiB,
    taxiC,
    parkA
};

const int TAKEOFF_SIZE = 6;
const std::array <FlightSegment, 6> takeOff28 = {
    taxiC,
    taxiB,
    taxiD,
    exitOuter10_4,
    exitOuter10_3,
    exitOuter10_2
};

const std::array <FlightSegment, 6> takeOff10 = {
    taxiC,
    taxiB,
    taxiA,
    exitOuter28_4,
    exitOuter28_3,
    exitOuter28_2
};

