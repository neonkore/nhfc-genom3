#
# Copyright (c) 2015-2020 LAAS/CNRS
# All rights reserved.
#
# Redistribution and use  in source  and binary  forms,  with or without
# modification, are permitted provided that the following conditions are
# met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
#					Anthony Mallet on Tue Aug 11 2015
#
lib_LTLIBRARIES = libnhfc_codels.la

libnhfc_codels_la_SOURCES  =	nhfc_c_types.h
libnhfc_codels_la_SOURCES +=	nhfc_codels.c
libnhfc_codels_la_SOURCES +=	nhfc_main_codels.c
libnhfc_codels_la_SOURCES +=	nhfc_wo_codels.c

libnhfc_codels_la_SOURCES +=	controller.cc
libnhfc_codels_la_SOURCES +=	admittance.cc
libnhfc_codels_la_SOURCES +=	observer.cc
libnhfc_codels_la_SOURCES +=	codels.h

libnhfc_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libnhfc_codels_la_LIBADD   =	$(requires_LIBS)
libnhfc_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libnhfc_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libnhfc_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	nhfc_c_types.h
CLEANFILES=	${BUILT_SOURCES}
nhfc_c_types.h: ${top_srcdir}/nhfc.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/nhfc_c_types.h.d
