# coding: utf-8

import zencad
import zencad.lazifier 
import pyservoce
import evalcache
from pyservoce import Scene, View, Viewer, Color

import tempfile
import sys
import os

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from PIL import Image
import numpy as np

import re
import time
import threading
import signal
import runpy

#from OpenGL.GL import *
#from OpenGL.GLUT import *
#from OpenGL.GLU import *

import inotify.adapters
import math

text_editor = "subl"
main_window = None
started_by = None
diag = None
ensave = None
desave = None
onplace = None

ABOUT_TEXT = "CAD system for righteous zen programmers."
BANNER_TEXT = (#"\n"
			"███████╗███████╗███╗   ██╗ ██████╗ █████╗ ██████╗ \n"
			"╚══███╔╝██╔════╝████╗  ██║██╔════╝██╔══██╗██╔══██╗\n"
			"  ███╔╝ █████╗  ██╔██╗ ██║██║     ███████║██║  ██║\n"
			" ███╔╝  ██╔══╝  ██║╚██╗██║██║     ██╔══██║██║  ██║\n"
			"███████╗███████╗██║ ╚████║╚██████╗██║  ██║██████╔╝\n"
			"╚══════╝╚══════╝╚═╝  ╚═══╝ ╚═════╝╚═╝  ╚═╝╚═════╝ ")

QMARKER_MESSAGE = "Press 'Q' to set marker"
WMARKER_MESSAGE = "Press 'W' to set marker"
DISTANCE_DEFAULT_MESSAGE = "Distance between markers"

def disable_lazy():
	global ensave, desave, onplace
	ensave = zencad.lazy.encache 
	desave = zencad.lazy.decache
	diag = zencad.lazy.diag
	onplace = zencad.lazy.onplace
	zencad.lazy.diag = False
	zencad.lazy.encache = False
	zencad.lazy.decache = False
	zencad.lazy.onplace = True

def restore_lazy():
	zencad.lazy.onplace = onplace
	zencad.lazy.encache = ensave
	zencad.lazy.decache = desave
	zencad.lazy.diag = diag

def show_label(lbl, en):
	if (en):
		lbl.setHidden(False)
	else:
		lbl.setHidden(True)


class MainWidget(QMainWindow):
	external_rerun_signal = pyqtSignal()

	def __init__(self, dispw):
		QMainWindow.__init__(self)
		self.cw = QWidget()
		self.dispw = dispw
		self.layout = QVBoxLayout()
		self.cpannellay = QHBoxLayout()
		self.infolay = QHBoxLayout()

		self.lastopened = None
		self.setWindowTitle("zenwidget");
		#self.setWindowIcon(QIcon(":/industrial-robot.svg"));

		self.layout.setSpacing(0)
		self.layout.setContentsMargins(0,0,0,0)

		#self.cp = QLabel("Cpannel test")
		#self.cp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed);
		#self.cp.setAlignment(Qt.AlignCenter)

		self.poslbl = QLabel("Tracking disabled")
		self.poslbl.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed);
		self.poslbl.setAlignment(Qt.AlignCenter)

		self.marker1=(zencad.pyservoce.point3(0,0,0),False)
		self.marker2=(zencad.pyservoce.point3(0,0,0),False)

		self.marker1Label = QLabel(QMARKER_MESSAGE)
		self.marker1Label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed);
		self.marker1Label.setStyleSheet("QLabel { background-color : rgb(100,0,0); color : white; }");
		self.marker1Label.setAlignment(Qt.AlignCenter)

		self.marker2Label = QLabel(WMARKER_MESSAGE)
		self.marker2Label.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed);
		self.marker2Label.setStyleSheet("QLabel { background-color : rgb(0,100,0); color : white; }");
		self.marker2Label.setAlignment(Qt.AlignCenter)

		self.markerDistLabel = QLabel(DISTANCE_DEFAULT_MESSAGE)
		self.markerDistLabel.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed);
		self.markerDistLabel.setAlignment(Qt.AlignCenter)

		self.infoLabel = QLabel("")
		self.infoLabel.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Fixed);
		self.infoLabel.setAlignment(Qt.AlignCenter)
		show_label(self.infoLabel, False)

		self.infolay.addWidget(self.poslbl)
		self.infolay.addWidget(self.marker1Label)
		self.infolay.addWidget(self.marker2Label)
		self.infolay.addWidget(self.markerDistLabel)
		self.infolay.addWidget(self.infoLabel)

		#self.cpannellay.addWidget(self.cp)

		self.layout.addLayout(self.cpannellay)
		self.layout.addWidget(self.dispw)
		self.layout.addLayout(self.infolay)
		self.cw.setLayout(self.layout)

		self.setCentralWidget(self.cw)
		#self.overlay = Overlay(self.dispw)

		self.createActions();
		self.createMenus();
		self.createToolbars();

		self.dispw.intersectPointSignal.connect(self.poslblSlot)

	def rerun_label_on_slot(self):
		self.infoLabel.setText("Please wait... Мы тут работаем, понимаешь.")
		show_label(self.marker1Label,False)
		show_label(self.marker2Label,False)
		show_label(self.markerDistLabel,False)
		show_label(self.poslbl,False)
		show_label(self.infoLabel,True)
		self.repaint()
		
	def rerun_label_off_slot(self):
		show_label(self.marker1Label,True)
		show_label(self.marker2Label,True)
		show_label(self.markerDistLabel,True)
		show_label(self.poslbl,True)
		show_label(self.infoLabel,False)
		self.update()
		
	def poslblSlot(self, obj):
		if obj[1]:
			self.poslbl.setText("x:{:8.3f},  y:{:8.3f},  z:{:8.3f}".format(obj[0].x, obj[0].y, obj[0].z))
		else:
			self.poslbl.setText("")
			self.update()

	def create_action(self, text, action, tip, shortcut = None, checkbox = False):
		act = QAction(self.tr(text), self)
		act.setStatusTip(self.tr(tip))
		
		if shortcut is not None: 
			act.setShortcut(self.tr(shortcut))
		
		if not checkbox:
			act.triggered.connect(action)
		else:
			act.setCheckable(True)
			act.toggled.connect(action)
		
		return act

	def createActions(self):
		self.mOpenAction = 	self.create_action("Open", 				self.openAction, 				"Open", 										"Ctrl+O")
		self.mTEAction = 	self.create_action("Open in Editor", 	self.externalTextEditorOpen, 	"Editor", 										"Ctrl+E")
		self.mExitAction = 	self.create_action("Exit", 				self.close, 					"Exit", 										"Ctrl+Q")
		self.mStlExport = 	self.create_action("Export STL...", 	self.exportStlAction, 			"Export file with external STL-Mesh format")
		self.mToFreeCad= 	self.create_action("To FreeCad", 		self.to_freecad_action, 		"Save temporary BRep representation and save FreeCad script to clipboard to load it")
		self.mBrepExport = 	self.create_action("Export BREP...", 	self.exportBrepAction, 			"Export file in BREP format")
		self.mScreen = 		self.create_action("Screenshot...", 	self.screenshotAction, 			"Do screen...")
		self.mAboutAction = self.create_action("About", 			self.aboutAction, 				"About the application")
		self.mReset = 		self.create_action("Reset", 			self.resetAction, 				"Reset")
		self.mCentering = 	self.create_action("Centering", 		self.centeringAction, 			"Centering")
		self.mAutoscale = 	self.create_action("Autoscale", 		self.autoscaleAction, 			"Autoscale", 									"Ctrl+A")
		self.mOrient1 = 	self.create_action("Axinometric view", 	self.orient1, 					"Orient1")
		self.mOrient2 = 	self.create_action("Free rotation view",self.orient2, 					"Orient2")
		self.mTracking = 	self.create_action("Tracking", 			self.trackingAction, 			"Tracking",				checkbox=True)
		self.mTestAction = 	self.create_action("TestAction", 		self.testAction, 				"TestAction")
		self.mInvalCache = 	self.create_action("Invalidate cache", 	self.invalidateCacheAction, 	"Invalidate cache")
		self.mCacheInfo = 	self.create_action("Cache info", 		self.cacheInfoAction, 			"Cache info")
		self.mDebugInfo = 	self.create_action("Debug info", 		self.debugInfoAction, 			"Debug info")

	def _add_open_action(self, menu, name, path):
		def callback():
			self._open_routine(path)

		menu.addAction(self.create_action(name, callback, path))

	def _init_example_menu(self, menu, directory):
		files = os.listdir(directory)
		scripts = [f for f in files if os.path.splitext(f)[1] == ".py"]
		dirs = [f for f in files if os.path.isdir(os.path.join(directory, f)) and f != "__pycache__" and f != "fonts"]
		
		for f in sorted(scripts):
			self._add_open_action(menu, f, os.path.join(directory, f))

		for d in sorted(dirs):
			m = menu.addMenu(d)
			self._init_example_menu(m, os.path.join(directory, d))	

	def createMenus(self):
		self.mFileMenu = self.menuBar().addMenu(self.tr("&File"))
		self.mFileMenu.addAction(self.mOpenAction)
		self.mFileMenu.addAction(self.mTEAction)
		self.exampleMenu = self.mFileMenu.addMenu("Examples")
		self.mFileMenu.addAction(self.mStlExport)
		self.mFileMenu.addAction(self.mBrepExport)
		self.mFileMenu.addAction(self.mToFreeCad)
		self.mFileMenu.addAction(self.mScreen)
		self.mFileMenu.addSeparator()
		self.mFileMenu.addAction(self.mExitAction)

		moduledir = os.path.dirname(__file__)
		self._init_example_menu(self.exampleMenu, os.path.join(moduledir, "examples"))
	
		self.mNavigationMenu = self.menuBar().addMenu(self.tr("&Navigation"))
		self.mNavigationMenu.addAction(self.mReset)
		self.mNavigationMenu.addAction(self.mCentering)
		self.mNavigationMenu.addAction(self.mAutoscale)
		self.mNavigationMenu.addAction(self.mOrient1)
		self.mNavigationMenu.addAction(self.mOrient2)
		self.mNavigationMenu.addAction(self.mTracking)
	
		self.mUtilityMenu = self.menuBar().addMenu(self.tr("&Utility"))
		self.mUtilityMenu.addAction(self.mCacheInfo)
		self.mUtilityMenu.addSeparator()
		self.mUtilityMenu.addAction(self.mInvalCache)

		self.mHelpMenu = self.menuBar().addMenu(self.tr("&Help"))
		self.mHelpMenu.addAction(self.mAboutAction)

		self.mHelpMenu = self.menuBar().addMenu(self.tr("&Devel"))
		self.mHelpMenu.addAction(self.mTestAction)
		self.mHelpMenu.addAction(self.mDebugInfo)
	
	def createToolbars(self):
		#self.btoolbar = QToolBar()
		#self.lbl = QLabel("HelloWorld")
		#self.btoolbar.addWidget(self.lbl)
		#self.addToolBar(Qt.BottomToolBarArea, self.btoolbar)
		pass

	def to_freecad_action(self):
		tmpfl = tempfile.mktemp(".brep")
		print(tmpfl)
		cb = QApplication.clipboard()
		cb.clear(mode=cb.Clipboard )
		cb.setText('import Part; export = Part.Shape(); export.read("{}"); Part.show(export); Gui.activeDocument().activeView().viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")'.format(tmpfl), mode=cb.Clipboard)
		pyservoce.brep_write(self.dispw.scene[0].shape(), tmpfl)
		QMessageBox.information(self, self.tr("ToFreeCad"),
			self.tr("Script copied to clipboard"));		


	def exportStlAction(self):
		d, okPressed = QInputDialog.getDouble(self, "Get double","Value:", 0.01, 0, 10, 10)
		if not okPressed:
			return

		filters = "*.stl;;*.*";
		defaultFilter = "*.stl";

		path = QFileDialog.getSaveFileName(self, "STL Export", 
			QDir.currentPath(),
			filters, defaultFilter);

		path = path[0]

		pyservoce.make_stl(self.dispw.scene[0].shape(), path, d)

	def trackingAction(self, en):
		if en:
			self.dispw.nointersect = False
		else:
			self.dispw.nointersect = True
			self.poslbl.setText("Tracking disabled")

	def externalTextEditorOpen(self):
		os.system(text_editor + " " + started_by)

	def testAction(self):
		pass

	def debugInfoAction(self):
		msgBox = QMessageBox(self)
		msgBox.setWindowTitle("Debug Info")
		msgBox.setWindowModality(Qt.WindowModal);
		msgBox.setInformativeText(
			"start_by: {}"
			.format(
				started_by
			))
		msgBox.exec();

	def exportBrepAction(self):
		filters = "*.brep;;*.*";
		defaultFilter = "*.brep";

		path = QFileDialog.getSaveFileName(self, "BREP Export", 
			QDir.currentPath(),
			filters, defaultFilter);

		path = path[0]

		pyservoce.brep_write(self.dispw.scene[0].shape(), path)

	def autoscaleAction(self):
		self.dispw.view.fit_all()

	def centeringAction(self):
		self.dispw.view.centering()

	def orient1(self):
		self.dispw.reset_orient1()

	def orient2(self):
		self.dispw.reset_orient2()

	def resetAction(self):
		self.dispw.psi =   math.cos(math.pi / 4)
		self.dispw.phi = - math.cos(math.pi / 4)
		self.dispw.view.autoscale()
		self.orient1()
		self.dispw.set_orient1()
		self.dispw.view.redraw()

	def invalidateCacheAction(self):
		files = zencad.lazy.cache.keys()
		for f in zencad.lazy.cache.keys():
			del zencad.lazy.cache[f]
		print("Invalidate cache: %d files removed" % len(files))

	def cacheInfoAction(self):
		def get_size(start_path = '.'):
			total_size = 0
			for dirpath, dirnames, filenames in os.walk(start_path):
				for f in filenames:
					fp = os.path.join(dirpath, f)
					total_size += os.path.getsize(fp)
			return total_size

		def sizeof_fmt(num, suffix='B'):
			for unit in ['','Ki','Mi','Gi','Ti','Pi','Ei','Zi']:
				if abs(num) < 1024.0:
					return "%3.1f%s%s" % (num, unit, suffix)
				num /= 1024.0
			return "%.1f%s%s" % (num, 'Yi', suffix)

		msgBox = QMessageBox(self)
		msgBox.setWindowTitle("Cache Info")
		msgBox.setWindowModality(Qt.WindowModal);
		msgBox.setInformativeText(
			"Path: {}"
			"<p>Hashing algorithm: {}"
			"<p>Files: {}"
			"<p>Size: {}".format(
				zencad.lazifier.cachepath,
				zencad.lazy.algo().name,
				len(zencad.lazy.cache.keys()),
				sizeof_fmt(get_size(zencad.lazifier.cachepath))
			))
		msgBox.exec();

	
	def updateDistLabel(self):
		qx,qy,qz = self.marker1[0].x, self.marker1[0].y, self.marker1[0].z
		wx,wy,wz = self.marker2[0].x, self.marker2[0].y, self.marker2[0].z
		xx,yy,zz = wx-qx, wy-qy, wz-qz
		dist = math.sqrt(xx**2 + yy**2 + zz**2)
		if self.marker1[1] or self.marker2[1]:
			self.markerDistLabel.setText("Distance: {:8.3f}".format(dist))
		else:
			self.markerDistLabel.setText(DISTANCE_DEFAULT_MESSAGE)	

	def screenshotAction(self):
		filters = "*.png;;*.bmp;;*.jpg;;*.*";
		defaultFilter = "*.png";

		path = QFileDialog.getSaveFileName(self, "Dump image", 
			QDir.currentPath(),
			filters, defaultFilter);

		path = path[0]
		
		w = self.dispw.width()
		h = self.dispw.height()

		raw = self.dispw.view.rawarray(w,h)
		npixels = np.reshape(np.asarray(raw), (h,w,3))
		nnnpixels = np.flip(npixels, 0).reshape((w * h * 3))

		rawiter = iter(nnnpixels)
		pixels = list(zip(rawiter, rawiter, rawiter))
		
		image = Image.new("RGB", (w, h))
		image.putdata(pixels)

		image.save(path)

	def _open_routine(self, path):
		#Проверяем, чтобы в файле был хоть намек на zencad...
		#А то чего его открывать.
		global started_by
		filetext = open(path).read()
		repattern1 = re.compile(r"import *zencad|from *zencad *import")
		
		zencad_search = repattern1.search(filetext)
		if zencad_search is None:
			print("no zencad import here... hm...")
			ret = QMessageBox.warning(self, self.tr("ZenCad file?"),
				self.tr("I can not find any zencad import here"))
			return

		print("rerun_label_on_slot()")
		self.rerun_label_on_slot()
		self.lastopened = path
		started_by = path
		self.external_rerun_signal.emit()

	def openAction(self):
		filters = "*.py;;*.*";
		defaultFilter = "*.py";

		startpath = QDir.currentPath() if self.lastopened == None else os.path.dirname(self.lastopened)

		path = QFileDialog.getOpenFileName(self, "Open File", 
			startpath,
			filters, defaultFilter)

		if path[1] == False:
			return

		self._open_routine(path[0])

	def aboutAction(self):
		QMessageBox.about(self, self.tr("About ZenCad Shower"),
			("<p>Widget for display zencad geometry."
			"<pre>{}\n"
			"{}\n"
			"Based on OpenCascade geometric core.<pre/>"
			"<p><h3>Feedback</h3>"
			"<pre>email: mirmikns@yandex.ru\n"
			"github: https://github.com/mirmik/zencad\n"
			"2018-2019<pre/>".format(BANNER_TEXT, ABOUT_TEXT)));

	def rerun_context(self, scn):
		#old_central_widget = self.dispw
		#old_central_widget.deleteLater()
		#self.dispw = DisplayWidget(scn, True, True)
		#self.setCentralWidget(self.dispw)
		self.dispw.viewer.clean_context()
		self.dispw.viewer.set_triedron_axes()
		self.dispw.viewer.add_scene(scn)
		self.dispw.scene = scn
		self.dispw.view.redraw()

	def keyPressEvent (self, event):
		if event.key() == Qt.Key_Q:
			self.marker1 = self.dispw.view.intersect_point(self.dispw.lastPosition.x(), self.dispw.lastPosition.y())
			x = self.marker1[0].x
			y = self.marker1[0].y
			z = self.marker1[0].z
			
			if self.marker1[1]:
				self.marker1Label.setText("x:{:8.3f},  y:{:8.3f},  z:{:8.3f}".format(x,y,z))
				print("Q: x:{0:8.3f},  y:{1:8.3f},  z:{2:8.3f} -> point3({0:.3f},{1:.3f},{2:.3f})".format(x,y,z))
			else:
				self.marker1Label.setText(QMARKER_MESSAGE)	
			self.updateDistLabel()

			if self.dispw.showmarkers:
				disable_lazy()
				self.dispw.MarkerQController.set_location(zencad.translate(x,y,z))
				restore_lazy()
				self.dispw.MarkerQController.hide(not self.marker1[1])
				self.dispw.view.redraw()
		
		if event.key() == Qt.Key_W:
			self.marker2 = self.dispw.view.intersect_point(self.dispw.lastPosition.x(), self.dispw.lastPosition.y())
			x = self.marker2[0].x
			y = self.marker2[0].y
			z = self.marker2[0].z

			if self.marker2[1]:
				self.marker2Label.setText("x:{:8.3f},  y:{:8.3f},  z:{:8.3f}".format(x,y,z))
				print("W: x:{0:8.3f},  y:{1:8.3f},  z:{2:8.3f} -> point3({0:.3f},{1:.3f},{2:.3f})".format(x,y,z))
			else:
				self.marker2Label.setText(WMARKER_MESSAGE)	
			self.updateDistLabel()
		
			if self.dispw.showmarkers:
				disable_lazy()
				self.dispw.MarkerWController.set_location(zencad.translate(x,y,z))
				restore_lazy()
				self.dispw.MarkerWController.hide(not self.marker2[1])
				self.dispw.view.redraw()
		
		if event.key() == Qt.Key_PageDown:
			self.dispw.pageDownKeyHandler()
		elif event.key() == Qt.Key_PageUp:
			self.dispw.pageUpKeyHandler()

class DisplayWidget(QWidget):
	intersectPointSignal = pyqtSignal(tuple)

	def __init__(self, arg, nointersect, showmarkers):
		QWidget.__init__(self)
	
		self.orient = 1
		self.mousedown = False

		self.inited = False
		self.painted = False
		self.nointersect = nointersect
		self.showmarkers = showmarkers

		self.scene = arg
		self.temporary1 = QPoint()	
		self.psi =   math.cos(math.pi / 4)
		self.phi = - math.cos(math.pi / 4)

		self.setBackgroundRole( QPalette.NoRole )
		self.setAttribute(Qt.WA_PaintOnScreen, True) 
		self.setMouseTracking(True)

	def reset_orient1(self):		
		self.orient = 1
		self.psi =   math.cos(math.pi / 4)
		self.phi = - math.cos(math.pi / 4)
		self.view.reset_orientation()
		self.view.autoscale()

	def reset_orient2(self):		
		self.orient = 2
		#self.view.reset_orientation()
		

	def set_orient1(self):
		self.view.set_projection(
			math.cos(self.psi) * math.cos(self.phi), 
			math.cos(self.psi) * math.sin(self.phi), 
			math.sin(self.psi)
		);

	def paintEvent(self, ev):
#		print("paintEvent")
		if self.inited and not self.painted:
			self.view.fit_all()
			self.view.must_be_resized()
			self.painted = True

	def showEvent(self, ev):
#		print("showEvent")
		if self.showmarkers:
			disable_lazy()
			self.msphere = zencad.sphere(1)
			self.MarkerQController = self.scene.add(self.msphere, zencad.Color(1,0,0))
			self.MarkerWController = self.scene.add(self.msphere, zencad.Color(0,1,0))
			restore_lazy()

		self.viewer = zencad.Viewer(self.scene)
		self.view = self.viewer.create_view()
		self.view.set_window(self.winId())
		self.view.set_gradient()
		
		self.set_orient1()
		self.view.set_triedron()
		self.viewer.set_triedron_axes()

		if self.showmarkers:
			self.MarkerQController.hide(True)
			self.MarkerWController.hide(True)

		self.inited = True

	def resizeEvent(self, ev):
		if self.inited:
			self.view.must_be_resized()

	def paintEngine(self):
		return None

	def onLButtonDown(self, theFlags, thePoint):
		self.temporary1 = thePoint;
		self.view.start_rotation(thePoint.x(), thePoint.y(), 1)

	def onRButtonDown(self, theFlags, thePoint):
		self.temporary1 = thePoint;

	def onMButtonDown(self, theFlags, thePoint):
		self.temporary1 = thePoint;

	def onMouseWheel(self, theFlags, theDelta, thePoint):
		aFactor = 16
	
		aX = thePoint.x()
		aY = thePoint.y()
	
		if (theDelta.y() > 0):
			aX += aFactor
			aY += aFactor
		else:
			aX -= aFactor
			aY -= aFactor
		
		self.view.zoom(thePoint.x(), thePoint.y(), aX, aY)

	def onMouseMove(self, theFlags, thePoint):
		mv = thePoint - self.temporary1
		self.temporary1 = thePoint

		self.lastPosition = thePoint

		modifiers = QApplication.keyboardModifiers()
		
		if modifiers == Qt.ShiftModifier:
			self.shift_pressed = True			
		else:
			self.shift_pressed = False


		if modifiers == Qt.AltModifier:
			if not self.alt_pressed:
				self.view.start_rotation(thePoint.x(), thePoint.y(), 1)
			self.alt_pressed = True			
		else:
			self.alt_pressed = False
			

		if not self.nointersect and not self.mousedown:
			ip = self.view.intersect_point(thePoint.x(), thePoint.y())
			self.intersectPointSignal.emit(ip)

		if theFlags & Qt.LeftButton or self.alt_pressed: 
			if self.orient == 1:  
				self.phi -= mv.x() * 0.01;
				self.psi += mv.y() * 0.01;
				if self.psi > math.pi*0.4999: self.psi = math.pi*0.4999
				if self.psi < -math.pi*0.4999: self.psi = -math.pi*0.4999
				self.set_orient1()
		
			if self.orient == 2:
					self.view.rotation(thePoint.x(), thePoint.y());
		
		if theFlags & Qt.RightButton or self.shift_pressed:
			self.view.pan(mv.x(), -mv.y())
	
	def wheelEvent(self, e):
		self.onMouseWheel(e.buttons(), e.angleDelta(), e.pos());

	def mouseReleaseEvent(self, e):
		pass

	def mouseMoveEvent(self, e):
		self.onMouseMove(e.buttons(), e.pos());

	def mouseReleaseEvent(self, e):
		self.mousedown = False

	def mousePressEvent(self, e):
		self.mousedown = True
		if e.button() == Qt.LeftButton:
			self.onLButtonDown((e.buttons() | e.modifiers()), e.pos())
		
		elif e.button() == Qt.MidButton:
			self.onMButtonDown((e.buttons() | e.modifiers()), e.pos())
		
		elif e.button() == Qt.RightButton:
			self.onRButtonDown((e.buttons() | e.modifiers()), e.pos())

	def pageDownKeyHandler(self):
		x = self.width()/2
		y = self.height()/2
		factor = 16
		self.view.zoom(x, y, x - factor, y - factor)

	def pageUpKeyHandler(self):
		x = self.width()/2
		y = self.height()/2
		factor = 16
		self.view.zoom(x, y, x + factor, y + factor)

class update_loop(QThread):
	def __init__(self, parent, updater_function, wdg, pause_time=0.01):
		QThread.__init__(self, parent)
		self.updater_function = updater_function 
		self.wdg = wdg
		self.pause_time = pause_time

	def run(self):
		while 1:
			ensave = zencad.lazy.encache 
			desave = zencad.lazy.decache
			onplace = zencad.lazy.onplace
			diag = zencad.lazy.diag
			if self.wdg.inited:
				zencad.lazy.encache = False
				zencad.lazy.decache = False
				zencad.lazy.onplace = True
				zencad.lazy.diag = False
				self.updater_function(self.wdg.viewer)
				zencad.lazy.onplace = onplace
				zencad.lazy.encache = ensave
				zencad.lazy.decache = desave
				zencad.lazy.diag = diag
				time.sleep(self.pause_time)

class rerun_notify_thread(QThread):
	rerun_label_on_signal = pyqtSignal()
	rerun_label_off_signal = pyqtSignal()
	external_autoscale_signal = pyqtSignal()

	def __init__(self, parent):
		QThread.__init__(self, parent)

	def init_notifier(self, path):
		self.notifier = inotify.adapters.Inotify()
		self.notifier.add_watch(path)

	def run(self):
		self.restart = False
		
		try:
			while 1:
				self.init_notifier(started_by)
				for event in self.notifier.event_gen():
					if event is not None:
						if 'IN_CLOSE_WRITE' in event[1]:
							print("started_by was rewrited. try use rerun")
							self.rerun_label_on_signal.emit()
							self.rerun()
					if self.restart:
						self.restart = False
						break
		except Exception as e:
			print("Warning: Rerun thread was finished:", e)
			
	def externalRerun(self):
		self.restart=True
		self.rerun()
		self.external_autoscale_signal.emit()
		
	def rerun(self):
		global default_scene
		zencad.shower.show_impl = zencad.shower.update_show
		default_scene=Scene()
		
		path = os.path.abspath(started_by)
		
		def routine():
			syspath = os.path.dirname(path)
			sys.path.insert(0, syspath)
	
			try:
				#self.rerun_label_on_signal.emit()
				zencad.lazifier.restore_default_lazyopts()
				#exec(open(started_by).read(), glbls)
				file_globals = runpy.run_path(path, run_name="__main__")
				print("Rerun finished correctly")
			except Exception as e:
				print("Error: Exception catched in rerun: type:{} text:{}".format(e.__class__.__name__, e))
	
			sys.path.remove(syspath)
			self.rerun_label_off_signal.emit()

		do = QThread(routine)
		do.start()

##display
default_scene = Scene()

def display(shp, color = Color(0.6, 0.6, 0.8)):
	if isinstance(shp, evalcache.LazyObject):
		return default_scene.add(evalcache.unlazy(shp), color)
	else:
		return default_scene.add(shp, color)

def disp(*args,**kwargs): display(*args, **kwargs)

def highlight(m): return display(m, Color(0.5, 0, 0, 0.5))
def hl(m) : return highlight(m)

def show(scene=None, animate = None, pause_time = 0.01, nointersect=True, showmarkers=True):
	if scene is None: scene = default_scene
	return show_impl(scene, animate, pause_time, nointersect, showmarkers)

def show_impl(scene, animate, pause_time, nointersect, showmarkers):
	global started_by
	global main_window
	started_by = sys.argv[0] if os.path.basename(sys.argv[0]) != "zencad" else os.path.join(zencad.moduledir, "__main__.py")

	app = QApplication(sys.argv)
	#app.lastWindowClosed.connect(app.quit)
	app.lastWindowClosed.connect(sys.exit)

	app.setWindowIcon(QIcon(os.path.dirname(__file__) + '/industrial-robot.svg'))

	fmt = QSurfaceFormat()
	fmt.setDepthBufferSize(24)
	fmt.setStencilBufferSize(8)
	fmt.setVersion(3, 2)
	fmt.setProfile(QSurfaceFormat.CoreProfile)
	QSurfaceFormat.setDefaultFormat(fmt)

	disp = DisplayWidget(scene, nointersect, showmarkers)
	main_window = MainWidget(disp);	
	main_window.resize(800,600)

	if animate != None:
		thr = update_loop(main_window, animate, disp, pause_time)
		thr.start()
		#thr = QThread(update_loop, animate, disp, update_time)
		#thr.start()

	thr_notify = rerun_notify_thread(main_window)
	thr_notify.start()
	

	#def sigint_handler(*args):
	#	sys.exit(-1)
	
	#signal.signal(signal.SIGINT, sigint_handler)

#	print("show")
	thr_notify.rerun_label_off_signal.connect(main_window.rerun_label_off_slot)
	thr_notify.rerun_label_on_signal.connect(main_window.rerun_label_on_slot)
	thr_notify.external_autoscale_signal.connect(main_window.resetAction)
	main_window.external_rerun_signal.connect(thr_notify.externalRerun)

	main_window.show()

	return app.exec()

def update_show(scene, animate = None, pause_time = 0.01, nointersect=True, showmarkers=True):
	main_window.rerun_context(scene)