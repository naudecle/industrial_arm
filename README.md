# industrial_arm
# Modélisation d'un bras robotique: URDF / Xacro &amp; MoveIt 2


## Choix et justification de la représentation

Pour la modélisation de ce bras robotique industriel, une architecture modulaire Xacro (macros XML) a été privilégiée par rapport à un simple fichier URDF brut.

### 1. Critères du choix de Xacro

- Evalution mathématique: les données de positionnement visuel fournies pour ce bras s'appuient largement sur des expressions mathématiques et des constantes trigonométriques (par exemple, des formules d'orientation telles que rpy="0 -${PI/2} ${PI/2}" pour le bras forward_drive_arm). Un fichier URDF natif ne peut pas évaluer de variables ou d'expressions ; il analyse uniquement des nombres à virgule flottante absolus. L'utilisation de Xacro nous a permis de conserver ces expressions mathématiques exactes directement dans le fichier, ce qui a éliminé les erreurs de précalcul manuel.

- Gestion des propriétés globales: les spécifications du projet stipulent que tous les maillages STL externes sont surdimensionnés et nécessitent un facteur d'ajustement d'échelle global de 0,01 exactement sur les dimensions $X$, $Y$ et $Z$ pour s'afficher correctement. En URDF pur, ce paramètre d'échelle devrait être codé en dur manuellement dans les propriétés visuelles et de collision de chaque lien. Avec Xacro, il est déclaré une seule fois en tant que propriété globale (<xacro:property name="mesh_scale" value="0.01 0.01 0.01" />) et instancié de manière propre dans tout le document, garantissant ainsi la maintenabilité.

### 2. Avantages et desavantages du choix

**`Avantages`**

- Lisibilité et clarté du code : réduit le balisage XML redondant en isolant les valeurs répétitives (telles que les paramètres de liaison, les limites de sécurité et les échelles de conversion) dans des propriétés nommées.

- Modularité: si la conception mécanique change (par exemple, si une liaison physique est allongée ou si une autre échelle de servomoteur est utilisée), les modifications ne doivent être effectuées qu'à un seul endroit dans le code, au lieu de devoir rechercher les occurrences en double dans des centaines de lignes de code XML.

**`Desavantages`**

- La charge de compilation Xacro désigne le temps de traitement nécessaire à l'analyseur Xacro pour interpréter les macros et les propriétés en une chaîne XML URDF standard. Dans les modèles volumineux comportant des calculs complexes ou des boucles, cela peut entraîner une forte augmentation des temps de lancement. 

### 3. Cas d'un choix différent

On aurait opté pour un fichier URDF simple et unique si le robot avait été une structure minimaliste et rigide, composée uniquement de formes primitives de base (comme un simple boîtier de fixation de capteur fixe ou une configuration cartésienne basique à deux articulations), sans mise à l'échelle complexe des maillages, sans structures articulées symétriques et avec des décalages spatiaux entièrement statiques et précalculés.