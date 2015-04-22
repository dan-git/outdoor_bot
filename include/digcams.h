// helper functions for camera control
// this comes from the samples and examples in gphoto2

#ifndef CAMCONTROL_H
#define CAMCONTROL_H

#include <ros/ros.h>
#include "outdoor_bot/digcams_custom.h"
#include "outdoor_bot/digcams_service.h"
#include "std_msgs/String.h"
#include <string>
#include <stdio.h>
#include <stdlib.h>  
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdarg.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
//#include <opencv3/imgproc/imgproc.hpp>
//#include <opencv3/highgui/highgui.hpp>
#include <gphoto2/gphoto2-camera.h> // libgphoto2-2 has to be installed to find this, not just gphoto2 

static GPPortInfoList		*portinfolist = NULL;
static CameraAbilitiesList	*abilities = NULL;

/*
 * This detects all currently attached cameras and returns
 * them in a list. It avoids the generic usb: entry.
 *
 * This function does not open nor initialize the cameras yet.
 */
int
sample_autodetect (CameraList *list, GPContext *context) {
	gp_list_reset (list);
        return gp_camera_autodetect (list, context);
}

/*
 * This function opens a camera depending on the specified model and port.
 */
int
sample_open_camera (Camera ** camera, const char *model, const char *port, GPContext *context) {
	int		ret, m, p;
	CameraAbilities	a;
	GPPortInfo	pi;

	ret = gp_camera_new (camera);
	if (ret < GP_OK) return ret;

	if (!abilities) {
		/* Load all the camera drivers we have... */
		ret = gp_abilities_list_new (&abilities);
		if (ret < GP_OK) return ret;
		ret = gp_abilities_list_load (abilities, context);
		if (ret < GP_OK) return ret;
	}

	/* First lookup the model / driver */
        m = gp_abilities_list_lookup_model (abilities, model);
	if (m < GP_OK) return ret;
        ret = gp_abilities_list_get_abilities (abilities, m, &a);
	if (ret < GP_OK) return ret;
        ret = gp_camera_set_abilities (*camera, a);
	if (ret < GP_OK) return ret;

	if (!portinfolist) {
		/* Load all the port drivers we have... */
		ret = gp_port_info_list_new (&portinfolist);
		if (ret < GP_OK) return ret;
		ret = gp_port_info_list_load (portinfolist);
		if (ret < 0) return ret;
		ret = gp_port_info_list_count (portinfolist);
		if (ret < 0) return ret;
	}

	/* Then associate the camera with the specified port */
        p = gp_port_info_list_lookup_path (portinfolist, port);
        switch (p) {
        case GP_ERROR_UNKNOWN_PORT:
                fprintf (stderr, "The port you specified "
                        "('%s') can not be found. Please "
                        "specify one of the ports found by "
                        "'gphoto2 --list-ports' and make "
                        "sure the spelling is correct "
                        "(i.e. with prefix 'serial:' or 'usb:').",
                                port);
                break;
        default:
                break;
        }
        if (p < GP_OK) return p;

        ret = gp_port_info_list_get_info (portinfolist, p, &pi);
        if (ret < GP_OK) return ret;
        ret = gp_camera_set_port_info (*camera, pi);
        if (ret < GP_OK) return ret;
	return GP_OK;
}

static void
ctx_error_func (GPContext *context, const char *str, void *data)
{
        fprintf  (stderr, "\n*** Contexterror ***              \n%s\n",str);
        fflush   (stderr);
}

static void
ctx_status_func (GPContext *context, const char *str, void *data)
{
        fprintf  (stderr, "%s\n", str);
        fflush   (stderr);
}

GPContext* sample_create_context() {
	GPContext *context;

	/* This is the mandatory part */
	context = gp_context_new();

	/* All the parts below are optional! */
        gp_context_set_error_func (context, ctx_error_func, NULL);
        gp_context_set_status_func (context, ctx_status_func, NULL);
 // set callbacks for camera messages
 // with c we don't have to cast the functions, but with cpp we do:
 //gp_context_set_error_func(context, error_func, NULL);
 //gp_context_set_message_func(context, message_func, NULL);
 //gp_context_set_error_func(context, (GPContextErrorFunc) error_func, NULL);
 //gp_context_set_message_func(context, (GPContextStatusFunc) message_func, NULL);

	/* also:
	gp_context_set_cancel_func    (p->context, ctx_cancel_func,  p);
        gp_context_set_message_func   (p->context, ctx_message_func, p);
        if (isatty (STDOUT_FILENO))
                gp_context_set_progress_funcs (p->context,
                        ctx_progress_start_func, ctx_progress_update_func,
                        ctx_progress_stop_func, p);
	 */
	return context;
}

/*
 * This function looks up a label or key entry of
 * a configuration widget.
 * The functions descend recursively, so you can just
 * specify the last component.
 */

static int
_lookup_widget(CameraWidget*widget, const char *key, CameraWidget **child) {
	int ret;
	ret = gp_widget_get_child_by_name (widget, key, child);
	if (ret < GP_OK)
		ret = gp_widget_get_child_by_label (widget, key, child);
	return ret;
}

/* Gets a string configuration value.
 * This can be:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int
get_config_value_string (Camera *camera, const char *key, char **str, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret;
	char			*val;

	ret = gp_camera_get_config (camera, &widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_get_config failed: %d\n", ret);
		return ret;
	}
	ret = _lookup_widget (widget, key, &child);
	if (ret < GP_OK) {
		fprintf (stderr, "lookup widget failed: %d\n", ret);
		goto out;
	}

	/* This type check is optional, if you know what type the label
	 * has already. If you are not sure, better check. */
	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		fprintf (stderr, "widget get type failed: %d\n", ret);
		goto out;
	}
	switch (type) {
        case GP_WIDGET_MENU:
        case GP_WIDGET_RADIO:
        case GP_WIDGET_TEXT:
		break;
	default:
		fprintf (stderr, "widget has bad type %d\n", type);
		ret = GP_ERROR_BAD_PARAMETERS;
		goto out;
	}

	/* This is the actual query call. Note that we just
	 * a pointer reference to the string, not a copy... */
	ret = gp_widget_get_value (child, &val);
	if (ret < GP_OK) {
		fprintf (stderr, "could not query widget value: %d\n", ret);
		goto out;
	}
	/* Create a new copy for our caller. */
	*str = strdup (val);
out:
	gp_widget_free (widget);
	return ret;
}


/* Sets a string configuration value.
 * This can set for:
 *  - A Text widget
 *  - The current selection of a Radio Button choice
 *  - The current selection of a Menu choice
 *
 * Sample (for Canons eg):
 *   get_config_value_string (camera, "owner", &ownerstr, context);
 */
int
set_config_value_string (Camera *camera, const char *key, const char *val, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret;

	ret = gp_camera_get_config (camera, &widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_get_config failed: %d\n", ret);
		return ret;
	}
	ret = _lookup_widget (widget, key, &child);
	if (ret < GP_OK) {
		fprintf (stderr, "lookup widget failed: %d\n", ret);
		goto out;
	}

	/* This type check is optional, if you know what type the label
	 * has already. If you are not sure, better check. */
	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		fprintf (stderr, "widget get type failed: %d\n", ret);
		goto out;
	}
	switch (type) {
        case GP_WIDGET_MENU:
        case GP_WIDGET_RADIO:
        case GP_WIDGET_TEXT:
		break;
	default:
		fprintf (stderr, "widget has bad type %d\n", type);
		ret = GP_ERROR_BAD_PARAMETERS;
		goto out;
	}

	/* This is the actual set call. Note that we keep
	 * ownership of the string and have to free it if necessary.
	 */
	ret = gp_widget_set_value (child, val);
	if (ret < GP_OK) {
		fprintf (stderr, "could not set widget value: %d\n", ret);
		goto out;
	}
	/* This stores it on the camera again */
	ret = gp_camera_set_config (camera, widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_set_config failed: %d\n", ret);
		return ret;
	}
out:
	gp_widget_free (widget);
	return ret;
}


/*
 * This enables/disables the specific canon capture mode.
 * 
 * For non canons this is not required, and will just return
 * with an error (but without negative effects).
 */
int
canon_enable_capture (Camera *camera, int onoff, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret;

	ret = gp_camera_get_config (camera, &widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_get_config failed: %d\n", ret);
		return ret;
	}
	ret = _lookup_widget (widget, "capture", &child);
	if (ret < GP_OK) {
		/*fprintf (stderr, "lookup widget failed: %d\n", ret);*/
		goto out;
	}

	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		fprintf (stderr, "widget get type failed: %d\n", ret);
		goto out;
	}
	switch (type) {
        case GP_WIDGET_TOGGLE:
		break;
	default:
		fprintf (stderr, "widget has bad type %d\n", type);
		ret = GP_ERROR_BAD_PARAMETERS;
		goto out;
	}
	/* Now set the toggle to the wanted value */
	ret = gp_widget_set_value (child, &onoff);
	if (ret < GP_OK) {
		fprintf (stderr, "toggling Canon capture to %d failed with %d\n", onoff, ret);
		goto out;
	}
	/* OK */
	ret = gp_camera_set_config (camera, widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_set_config failed: %d\n", ret);
		return ret;
	}
out:
	gp_widget_free (widget);
	return ret;
}


/* calls the Nikon DSLR or Canon DSLR autofocus method. */
int
camera_auto_focus(Camera *camera, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret,val;

	ret = gp_camera_get_config (camera, &widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_get_config failed: %d\n", ret);
		return ret;
	}
	ret = _lookup_widget (widget, "autofocusdrive", &child);
	if (ret < GP_OK) {
		fprintf (stderr, "lookup 'autofocusdrive' failed: %d\n", ret);
		goto out;
	}

	/* check that this is a toggle */
	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		fprintf (stderr, "widget get type failed: %d\n", ret);
		goto out;
	}
	switch (type) {
        case GP_WIDGET_TOGGLE:
		break;
	default:
		fprintf (stderr, "widget has bad type %d\n", type);
		ret = GP_ERROR_BAD_PARAMETERS;
		goto out;
	}

	ret = gp_widget_get_value (child, &val);
	if (ret < GP_OK) {
		fprintf (stderr, "could not get widget value: %d\n", ret);
		goto out;
	}
	val++;
	ret = gp_widget_set_value (child, &val);
	if (ret < GP_OK) {
		fprintf (stderr, "could not set widget value to 1: %d\n", ret);
		goto out;
	}

	ret = gp_camera_set_config (camera, widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "could not set config tree to autofocus: %d\n", ret);
		goto out;
	}
out:
	gp_widget_free (widget);
	return ret;
}


/* Manual focusing a camera...
 * xx is -3 / -2 / -1 / 0 / 1 / 2 / 3
 */
int
camera_manual_focus (Camera *camera, int xx, GPContext *context) {
	CameraWidget		*widget = NULL, *child = NULL;
	CameraWidgetType	type;
	int			ret;
	float			rval;
	char			*mval;

	ret = gp_camera_get_config (camera, &widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "camera_get_config failed: %d\n", ret);
		return ret;
	}
	ret = _lookup_widget (widget, "manualfocusdrive", &child);
	if (ret < GP_OK) {
		fprintf (stderr, "lookup 'manualfocusdrive' failed: %d\n", ret);
		goto out;
	}

	/* check that this is a toggle */
	ret = gp_widget_get_type (child, &type);
	if (ret < GP_OK) {
		fprintf (stderr, "widget get type failed: %d\n", ret);
		goto out;
	}
	switch (type) {
        case GP_WIDGET_RADIO: {
		int choices = gp_widget_count_choices (child);

		ret = gp_widget_get_value (child, &mval);
		if (ret < GP_OK) {
			fprintf (stderr, "could not get widget value: %d\n", ret);
			goto out;
		}
		if (choices == 7) { /* see what Canon has in EOS_MFDrive */
			ret = gp_widget_get_choice (child, xx+4, (const char**)&mval);
			if (ret < GP_OK) {
				fprintf (stderr, "could not get widget choice %d: %d\n", xx+2, ret);
				goto out;
			}
			fprintf(stderr,"manual focus %d -> %s\n", xx, mval);
		}
		ret = gp_widget_set_value (child, mval);
		if (ret < GP_OK) {
			fprintf (stderr, "could not set widget value to 1: %d\n", ret);
			goto out;
		}
		break;
	}
        case GP_WIDGET_RANGE:
		ret = gp_widget_get_value (child, &rval);
		if (ret < GP_OK) {
			fprintf (stderr, "could not get widget value: %d\n", ret);
			goto out;
		}
	
		switch (xx) { /* Range is on Nikon from -32768 <-> 32768 */
		case -3:	rval = -1024;break;
		case -2:	rval =  -512;break;
		case -1:	rval =  -128;break;
		case  0:	rval =     0;break;
		case  1:	rval =   128;break;
		case  2:	rval =   512;break;
		case  3:	rval =  1024;break;

		default:	rval = xx;	break; /* hack */
		}

		fprintf(stderr,"manual focus %d -> %f\n", xx, rval);

		ret = gp_widget_set_value (child, &rval);
		if (ret < GP_OK) {
			fprintf (stderr, "could not set widget value to 1: %d\n", ret);
			goto out;
		}
		break;
	default:
		fprintf (stderr, "widget has bad type %d\n", type);
		ret = GP_ERROR_BAD_PARAMETERS;
		goto out;
	}


	ret = gp_camera_set_config (camera, widget, context);
	if (ret < GP_OK) {
		fprintf (stderr, "could not set config tree to autofocus: %d\n", ret);
		goto out;
	}
out:
	gp_widget_free (widget);
	return ret;
}

/*
static void errordumper(GPLogLevel level, const char *domain, const char *str,
                 void *data) {
  fprintf(stdout, "%s\n", str);
}
*/

/*
static void
capture_to_file(Camera *camera, GPContext *context, char *fn) {
	int fd, retval;
	CameraFile *file;
	CameraFilePath camera_file_path;

	printf("Capturing.\n");

	//NOP: This gets overridden in the library to /capt0000.jpg 
	strcpy(camera_file_path.folder, "/");
	strcpy(camera_file_path.name, "foo.jpg");

	retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
	printf("  Retval: %d\n", retval);

	printf("Pathname on the camera: %s/%s\n", camera_file_path.folder, camera_file_path.name);

	fd = open(fn, O_CREAT | O_WRONLY, 0644);
	retval = gp_file_new_from_fd(&file, fd);
	printf("  Retval: %d\n", retval);
	retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name,
		     GP_FILE_TYPE_NORMAL, file, context);
	printf("  Retval: %d\n", retval);

	printf("Deleting.\n");
	retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name,
			context);
	printf("  Retval: %d\n", retval);

	gp_file_free(file);
}
*/
#endif // CAMCONTROL_H

