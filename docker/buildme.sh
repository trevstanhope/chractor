#!/bin/bash
cd resources && rm -f app.tar && tar -cvf app.tar app
cd -
docker build . -t chractor:latest
