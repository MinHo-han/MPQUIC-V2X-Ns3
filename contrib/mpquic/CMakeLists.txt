# SPDX-License-Identifier: GPL-2.0-only

build_lib(
  LIBNAME mpquic
  SOURCE_FILES
    model/mpquic-header.cc
    model/mpquic-path.cc
    model/mpquic-scheduler.cc
    model/mpquic-socket-base.cc
    model/mpquic-sender-app.cc
    model/mpquic-receiver-app.cc
    model/mpquic-video-sender-app.cc
    model/mpquic-inference-sender-app.cc
    helper/mpquic-helper.cc
  HEADER_FILES
    model/mpquic-header.h
    model/mpquic-path.h
    model/mpquic-scheduler.h
    model/mpquic-socket-base.h
    model/mpquic-sender-app.h
    model/mpquic-receiver-app.h
    model/mpquic-video-sender-app.h
    model/mpquic-inference-sender-app.h
    helper/mpquic-helper.h
  LIBRARIES_TO_LINK
    ${libcore}
    ${libnetwork}
    ${libinternet}
    ${libapplications}
)
