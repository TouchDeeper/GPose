//
// Created by wang on 19-4-29.
//
#include "Matching.h"
#include "parameters.h"
#include "config_reader.h"
int main(int argc, char** argv){

    ReadParameters("../config.ini");

    Matching match_pipeline;
    match_pipeline.LoadModel();
    match_pipeline.LoadScene();
    match_pipeline.ShowCandidate();



}
