BASE_DIR	= ../..
include $(BASE_DIR)/build/GNUMake/paths.mak
include $(BASE_DIR)/build/GNUMake/$(TOOLS_MAK)

SOURCES 		= LBRTorqueSineOverlayApp.cpp \
			 	  LBRTorqueSineOverlayClient.cpp

TARGET 		= LBRTorqueSineOverlay

CXXFLAGS 	+=
LDFLAGS 		+= $(LIB_DIR)/libFRIClient.a $(LIB_DIR)/hiredis/libhiredis.a $(LIB_DIR)/jsoncpp/build/lib/libjsoncpp.a

include $(BASE_DIR)/build/GNUMake/rules_examples.mak
