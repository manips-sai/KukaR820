BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

SOURCES 		= LBRTorqueOverlayApp.cpp \
			 	  LBRTorqueOverlayClient.cpp

TARGET 		= LBRTorqueOverlay

CXXFLAGS 	+=
LDFLAGS 		+= $(LIB_DIR)/libFRIClient.a $(LIB_DIR)/hiredis/libhiredis.a $(LIB_DIR)/jsoncpp/build/lib/libjsoncpp.a

include $(BASE_DIR)/build/GNUMake/rules_examples.mak
