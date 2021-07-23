#!/bin/bash

gst-launch-1.0 videotestsrc pattern=foo ! videoconvert ! autovideosink

