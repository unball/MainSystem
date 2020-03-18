from controller.vision import Vision
from controller.vision.visionMessage import VisionMessage
from model.vision.mainVisionModel import MainVisionModel
from controller.tools.pixel2metric import pixel2meters, meters2pixel, normToAbs
from controller.tools import norm
from view.tools.drawing import Drawing
import cv2
import numpy as np
import copy
import time

class MainVision(Vision):
  """Classe que implementa a visão principal da UnBall, que utiliza segmentação por única cor e faz a identificação por forma."""
  
  def __init__(self, world):
    super().__init__(world)
    
    self.__model = MainVisionModel()
    """Modelo `MainSystem.model.vision.mainVisionModel.MainVisionModel` que mantém as variáveis da visão"""
    
    self.__current_frame_shape = None
    """Mantém o formato do frame, caso mude a matriz de homografia será recalculada com base nos pontos (em coordenadas relativas) selecionados"""
    
    self.__homography_points_updated = False
    
    self.__angles = np.array([0, 90, 180, -90, -180])
    """Contém uma lista que corrige o ângulo do vetor que liga o centro de massa do detalhe ao centro de massa da camisa para o ângulo que o robô anda para frente"""

    self.lastMessage = None
    self.average = 0

  @property
  def preto_hsv(self):
    """Retorna o valor dos limites HSV para segmentação dos elementos"""
    return self.__model.preto_hsv
    
  @property
  def time_hsv(self):
    """Retorna o valor dos limites HSV para segmentação do time"""
    return self.__model.time_hsv
    
  @property
  def bola_hsv(self):
    """Retorna o valor dos limites HSV para segmentação da bola"""
    return self.__model.bola_hsv
    
  @property
  def use_homography(self):
    """Retorna a variável `use_homography` que indica se é para usar a matriz de homografia ou usar um corte retangular"""
    return self.__model.use_homography
  
  @property
  def areaRatio(self):
    """Retorna a variável `cont_rect_area_ratio` que contém a razão área triângulo/retângulo. Essa razão é usada para diferenciar as duas formas"""
    return self.__model.cont_rect_area_ratio
    
  @property
  def minInternalAreaContour(self):
    """Retorna a variável `min_internal_area_contour` que contém a área mínima do contorno interno aceitável. Se for detectado um contorno de área menor que essa, esse contorno será filtrado."""
    return self.__model.min_internal_area_contour
    
  @property
  def minExternalAreaContour(self):
    """Retorna a variável `min_external_area_contour` que contém a área mínima do contorno externo aceitável. Se for detectado um contorno de área menor que essa, esse contorno será filtrado."""
    return self.__model.min_external_area_contour

  def updateInternalPolygon(self, points):
    self.__model.internalPolygonPoints = points
  
  def atualizarPretoHSV(self, value, index):
    """Atualiza o valor do limite HSV de índice `index` para segmentação dos elementos."""
    self.__model.preto_hsv[index] = value
  
  def atualizarTimeHSV(self, value, index):
    """Atualiza o valor do limite HSV de índice `index` para segmentação do time."""
    self.__model.time_hsv[index] = value
  
  def atualizarBolaHSV(self, value, index):
    """Atualiza o valor do limite HSV de índice `index` para segmentação da bola."""
    self.__model.bola_hsv[index] = value
    
  def setUseHomography(self, value):
    """Atualiza o valor da flag `use_homography`."""
    self.__model.use_homography = value
    
  def updateCropPoints(self, points):
    """Atualiza os pontos de corte retangular."""
    self.__model.crop_points = points
    
  def updateHomographyPoints(self, points):
    """Atualiza os pontos chave para calcular a matriz de homografia."""
    self.__model.homography_points = points
    self.__homography_points_updated = True
  
  def atualizarAreaRatio(self, value):
    """Atualiza o parâmetro de área que diferencia retângulo de triângulo."""
    self.__model.cont_rect_area_ratio = value
    
  def atualizarMinInternalArea(self, value):
    """Atualiza o parâmetro de área mínima do contorno interno."""
    self.__model.min_internal_area_contour = value
    
  def atualizarMinExternalArea(self, value):
    """Atualiza o parâmetro de área mínima do contorno externo."""
    self.__model.min_external_area_contour = value
  
  def get_polygon_mask(frame, points):
      mask = np.zeros((*frame.shape[:2],1), np.uint8)
      if len(points) == 0: return cv2.bitwise_not(mask)
      pts = np.array([normToAbs(x[0], frame.shape) for x in points])
      cv2.fillConvexPoly(mask, pts, 255)
      return mask

  def crop(frame, p0, p1):
    """Método de classe que corta um frame dados dois pontos \\(p_0=(x_0,y_0)\\), \\(p_1=(x_1,y_1)\\) que definem as coordenadas inicial e final desde que \\(x_0<x_1\\) e \\(y_0<y_1\\)"""
    
    x, xf = (p0[0], p1[0])
    y, yf  = (p0[1], p1[1])
    return frame[y:yf, x:xf]
    
  def getHomography(self, shape):
    """Obtém a matriz de homografia"""
    if self.__homography_points_updated or self.__current_frame_shape != shape:
      if self.__model.homography_points is None:
        return None
      self.updateHomography(self.__model.homography_points, shape)
    return self.__model.homography
    
  def updateHomography(self, points, shape):
    """Atualiza a matriz de homografia com base nos pontos selecionados e no tamanho do frame"""
    height, width, _ = shape
    
    base = np.array([[0,0],[1,0],[1,1],[0,1]])
    key_points = np.array(points) * np.array([width, height])
    
    frame_points = base * np.maximum.reduce(key_points)
    
    h, mask = cv2.findHomography(key_points, frame_points, cv2.RANSAC)
    
    self.__model.homography = h.tolist()
    self.__homography_points_updated = False
    self.__current_frame_shape = shape
    self.__model.homography_points = points
    self.__homography = h
  
  def warp(self, frame):
    """Método que corta de forma retangular ou via homografia a depender de `MainSystem.model.vision.mainVisionModel.MainVisionModel`"""
    
    if not self.__model.use_homography:
      if self.__model.crop_points:
        p0 = normToAbs(self.__model.crop_points[0], frame.shape)
        p1 = normToAbs(self.__model.crop_points[1], frame.shape)
        return MainVision.crop(frame, p0, p1)
      else:
        return frame
        
    else:
      homography_matrix = self.getHomography(frame.shape)
      try:
        warpped = cv2.warpPerspective(frame, np.array(homography_matrix), (frame.shape[1], frame.shape[0]))
        key_points = np.array(self.__model.homography_points) * np.array([frame.shape[1], frame.shape[0]])
        extreme_points = np.maximum.reduce(key_points).astype(np.uint32)
        return MainVision.crop(warpped, (0,0), extreme_points)
      except:
        return frame
        
  def converterHSV(self, img):
    """Converte uma imagem RGB para HSV e aplica um filtro gaussiano"""
    img_filtered = cv2.GaussianBlur(img, (5,5), 0)
    return cv2.cvtColor(img_filtered, cv2.COLOR_BGR2HSV)
  
  def obterMascaraElementos(self,img):
    """Retorna uma máscara do que não é fundo"""
    fgMask = cv2.inRange(img, np.array(self.__model.preto_hsv[0:3]), np.array(self.__model.preto_hsv[3:6]))
    if len(self.__model.internalPolygonPoints) != 0: 
      fgMask &= MainVision.get_polygon_mask(img, self.__model.internalPolygonPoints)[:,:,0]
    return fgMask
    
  def obterMascaraTime(self, img):
    """Retorna uma máscara dos detalhes da camisa do time"""
    return cv2.inRange(img, np.array(self.__model.time_hsv[0:3]), np.array(self.__model.time_hsv[3:6]))
    
  def obterMascaraBola(self, img):
    """Retorna uma máscara do que é bola"""
    return cv2.inRange(img, np.array(self.__model.bola_hsv[0:3]), np.array(self.__model.bola_hsv[3:6]))
    
  def identificarBola(self, mask, shape, pref=(0,0)):
    """Com base em uma máscara, retorna posição em metros e raio da bola"""
    bolaContours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    bolaContours = [countor for countor in bolaContours if cv2.contourArea(countor) >= self.__model.min_internal_area_contour]

    if len(bolaContours) != 0:
      bolaContour = max(bolaContours, key=cv2.contourArea)
      ((x,y), radius) = cv2.minEnclosingCircle(bolaContour)
      return (pixel2meters(self._world, (x+pref[0],y+pref[1]), shape), radius)
    
    else: return None

  def identificarRobo(self, teamMask, componentMask, shape, pref=(0,0)):
    # Obtém dados do componente como uma camisa
    camisa = self.detectarCamisa(componentMask, shape, pref=pref)
    
    # Extrai dados da camisa
    if camisa is None: return None
    centro, centerMeters, angulo, camisaContours = camisa
    
    # Máscara dos componentes internos do elemento
    componentTeamMask = componentMask & teamMask
    
    # Tenta identificar um aliado
    aliado = self.detectarTime(componentTeamMask, centro, angulo, centerMeters, pref)
    if aliado is not None:
      return {
        "id": aliado[0],
        "pose": (*centerMeters, aliado[1]*np.pi/180),
        "debug": {"contornoInterno": aliado[2]}
      }
    
    # Adiciona camisa como adversário
    else:
      return {
        "id": None,
        "pose": (*centerMeters, angulo*np.pi/180),
        "debug": {"contornoExterno": camisaContours}
      }
    
  def aplicarFiltrosMorfologicos(self, mask):
    """Aplica filtros morfológicos com o objetivo de retirar ruido a uma mascara"""
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    filtered = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    filtered = cv2.dilate(filtered, kernel, iterations=1)
    return filtered
    
  def obterComponentesConectados(self, mask):
    """Retorna uma lista de máscaras de componentes conectados com base na máscara passada"""
    num_components, components = cv2.connectedComponents(mask)
    components = self.aplicarFiltrosMorfologicos(np.uint8(components))
    return [np.uint8(np.where(components == label, 255, 0)) for label in np.unique(components)[1:]]
    
  def detectarCamisa(self, component_mask, shape, pref=(0,0)):
    """Com base na máscara de um componente conectado extrai informação de posição e ângulo parcial de uma camisa"""
    # Encontra um contorno para a camisa com base no maior contorno
    mainContours,_ = cv2.findContours(component_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    mainContours = [countor + np.array(pref)[:2] for countor in mainContours if cv2.contourArea(countor)>=self.__model.min_external_area_contour]

    countMainContours = len(mainContours)
    
    # Contorno pequeno
    if countMainContours == 0:
      return None
      
    mainContour = max(mainContours, key=cv2.contourArea)
    
    # Encontra o menor retângulo que se inscreve no contorno
    rectangle = cv2.minAreaRect(mainContour)
    
    # Calcula a posição e ângulo parcial da camisa com base no retângulo
    center = rectangle[0]
    centerMeters = pixel2meters(self._world, np.array(center), shape)
    angle = rectangle[-1]
    
    return center, centerMeters, angle, mainContours
  
  def definePoly(self, countor):
    """Define se o contorno é mais parecido com um triângulo ou um retângulo"""
    rect = cv2.minAreaRect(countor)
    contourArea = cv2.contourArea(countor)
    rectArea = rect[1][0]*rect[1][1]
    
    return 4 if contourArea/rectArea > self.__model.cont_rect_area_ratio else 3
    
  def obterIdentificador(self, center, candidate):
    """
    .. todo:: Falta fazer usar o centro como base para saber se o candidato é ou não razoável
    Retorna o identificador com base no centro do robô e na identificação instantânea da camisa (devida somente ao frame atual) e retorna qual deve ser o identificador mais provável.
    """
    return candidate
    if not self.usePastPositions: return candidate
    
    nearestIdx = np.argmin([norm(x.raw_pos, center) for x in self._world.robots[:3]])
    return nearestIdx
  
  def detectarTime(self, componentTeamMask, center, rectangleAngle, centerMeters, pref=(0,0)):
    """Com base na máscara do detalhe do time extrai identifica qual é o robô aliado e obtém o ângulo total"""
    
    # Encontra os contornos internos com área maior que um certo limiar e ordena
    internalContours,_ = cv2.findContours(componentTeamMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    internalContours = [countor + np.array(pref)[:2] for countor in internalContours if cv2.contourArea(countor)>=self.__model.min_internal_area_contour]
    
    countInternalContours = len(internalContours)
    
    # Não é do nosso time
    if countInternalContours == 0:
      return None

    # Seleciona a forma principal
    mainShape = max(internalContours, key=cv2.contourArea)
    
    # Calcula o centro do contorno principal
    M = cv2.moments(mainShape)
    if M["m00"] == 0: return None
    
    cX = M["m10"] / M["m00"]
    cY = M["m01"] / M["m00"]
    # Define qual o polígono da figura principal
    poligono = self.definePoly(mainShape)
    
    # Computa o identificador com base na forma e no número de contornos internos
    candidato = (0 if poligono == 3 else 2) + countInternalContours -1
    if candidato >= self._world.n_robots: return None
    
    identificador = self.obterIdentificador(centerMeters, candidato)

    # Calcula o ângulo com base no vetor entre o centro do contorno principal e o centro da camisa
    calculatedAngle = 180.0/np.pi *np.arctan2(-(center[1]-cY), center[0]-cX)
    if identificador == 1:
      calculatedAngle = calculatedAngle - 45.0
    partialAngles =  -rectangleAngle + self.__angles
    estimatedAngle = partialAngles[np.abs(calculatedAngle - partialAngles).argmin()]
      
    
    return identificador, estimatedAngle, internalContours

  def cropBallRegion(self, img_hsv, lastMessage):
    ballPose = np.array([lastMessage.ball_x, lastMessage.ball_y])
    pmin = np.array(meters2pixel(self._world, ballPose-[0.1, -0.1], img_hsv.shape))
    pmax = np.array(meters2pixel(self._world, ballPose+[0.1, -0.1], img_hsv.shape))
    return {"region": MainVision.crop(img_hsv, pmin, pmax), "pmin": pmin}

  def cropRobotsRegion(self, img_hsv, lastMessage):
    regions = []
    for pose in lastMessage.allyPoses[:3]:
      robotPose = np.array([pose[0], pose[1]])
      pmin = np.array(meters2pixel(self._world, robotPose-[0.1, -0.1], img_hsv.shape))
      pmax = np.array(meters2pixel(self._world, robotPose+[0.1, -0.1], img_hsv.shape))
      regions.append({"region": MainVision.crop(img_hsv, pmin, pmax), "pmin": pmin})

    return regions

  def optimizedModeCondition(self):
    return self.lastMessage is not None and self.lastMessage.ball_found == True and sum([x[3] for x in self.lastMessage.allyPoses[:3]]) == 3

  def process(self, frame, checkpoint=None):
    # Verifica se todos os robôs+bola já foram identificados
    if self.optimizedModeCondition():
      print("OTIMIZADO!")
      self.lastMessage, frame = self.process_optimized(frame, self.lastMessage, checkpoint)
    
    # Executa o modo não otimizado
    else:
      self.lastMessage, frame = self.process_complete(frame, checkpoint)
    
    if checkpoint: return self.lastMessage, frame
    else: return self.lastMessage

  def process_complete(self, frame, checkpoint=None):
    # Corta a imagem
    img_warpped = self.warp(frame)

    # Muda o espaço de cor par HSV
    img_hsv = self.converterHSV(img_warpped)

    # Máscaras
    fgMask = self.obterMascaraElementos(img_hsv)
    teamMask = self.obterMascaraTime(img_hsv)
    ballMask = self.obterMascaraBola(img_hsv)

    # Mensagem de retorno
    message = VisionMessage(5)

    # Identifica a bola
    ball = self.identificarBola(ballMask, ballMask.shape)
    message.setBall(ball)

    # Máscara do que não é fundo nem bola
    fgMaskNoBall = cv2.bitwise_and(fgMask, cv2.bitwise_not(ballMask))

    # Obtém compomentes conectados na máscara (robos aliados e inimigos)
    components = self.obterComponentesConectados(fgMaskNoBall)

    # Identifica os robos
    for componentMask in components:
      robot = self.identificarRobo(teamMask, componentMask, componentMask.shape)

      # Robô não identificado
      if robot is None: continue
      
      # Aliado tem id
      if robot["id"] is not None:
        message.setRobot(robot["id"], robot["pose"], internalContours=robot["debug"]["contornoInterno"])
      
      # Inimigo
      else:
        message.setEnemyRobot(robot["pose"], extContour=robot["debug"]["contornoExterno"])

    return message, img_hsv

  def process_optimized(self, frame, lastMessage, checkpoint=None):
    # Corta a imagem
    img_warpped = self.warp(frame)

    # Muda o espaço de cor par HSV
    img_hsv = self.converterHSV(img_warpped)

    # Gera as sub-imagens
    ballRegions = self.cropBallRegion(img_hsv, lastMessage)
    robotRegions = self.cropRobotsRegion(img_hsv, lastMessage)

    # Mensagem de retorno começa como uma cópia da última mensagem
    message = copy.deepcopy(lastMessage)

    # Identifica a bola
    ballMask = self.obterMascaraBola(ballRegions["region"])
    ball = self.identificarBola(ballMask, img_hsv.shape, pref=ballRegions["pmin"])
    message.setBall(ball)

    # Identifica os robôs
    for robotId, region in enumerate(robotRegions):
      # Última pose
      lastpose = lastMessage.allyPoses[robotId]

      # Máscaras
      fgMask = self.obterMascaraElementos(region["region"])
      teamMask = self.obterMascaraTime(region["region"])
      ballMask = self.obterMascaraBola(region["region"])
      fgMaskNoBall = cv2.bitwise_and(fgMask, cv2.bitwise_not(ballMask))

      # Componentes conectados na região
      components = self.obterComponentesConectados(fgMaskNoBall)

      # Melhor candidato dos componentes
      bestCandidate = None

      # Para cada componente identifica
      for componentMask in components:
        robot = self.identificarRobo(teamMask, componentMask, img_hsv.shape, region["pmin"])

        # Robô não identificado
        if robot is None: continue

        # Robô é o procurado e é melhor candidato
        if robot["id"] == robotId and (bestCandidate is None or norm(robot["pose"], lastpose) < norm(bestCandidate["pose"], lastpose)):
          bestCandidate = robot

      # Não achou o robô
      if bestCandidate is None:
        message.unsetRobot(robotId)

      # Atualiza o robô na mensagem
      else:
        message.setRobot(robotId, bestCandidate["pose"], internalContours=bestCandidate["debug"]["contornoInterno"])

    return message, robotRegions[0]["region"]