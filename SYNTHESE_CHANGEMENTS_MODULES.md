# PX4 Autopilot Rocket - Synthèse des Changements de Modules

Ce document fournit une synthèse complète de tous les changements apportés à la base de code PX4 Autopilot Rocket depuis le commit `2249a5976ecb5e6b855951dde2b861ab5acc786a`, organisés par modules plutôt que par ordre chronologique.

**Période d'Analyse :** 11 août 2025 - 4 septembre 2025
**Total des Commits :** 6
**Auteur :** Noé Renevey <noe.renevey@edu.hefr.ch>
**Branche :** GRID-Rocket

---

## 🚀 Module Rocket Mode Manager
*Le module principal de contrôle de vol spécifique à la fusée*

### Fichiers Ajoutés:
- `src/modules/rocket_mode_manager/rocket_mode_manager.cpp` (568 → 397 → 314 → 36 → 30 lignes modifiées)
- `src/modules/rocket_mode_manager/rocket_mode_manager.hpp` (167 → 33 → 11 → 1 → 2 lignes modifiées)
- `src/modules/rocket_mode_manager/module.yaml` (119 → 121 → 24 → 30 → 5 lignes modifiées)
- `src/modules/rocket_mode_manager/CMakeLists.txt` (44 lignes ajoutées)
- `src/modules/rocket_mode_manager/Kconfig` (11 lignes ajoutées)

### Fonctionnalités Clés Implémentées:
- **Gestion complète des phases de vol de fusée** avec machine d'état pour les phases de vols.
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.hpp:98-104` - `enum class RocketState` définissant WAITING_LAUNCH, ROCKET_BOOST, ROCKET_COAST,...
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.hpp:110-113` - Méthodes de gestion des phases : `handle_waiting_launch_phase()`, `handle_rocket_boost_phase()`, `handle_rocket_coast_phase()`

- **Système de Détection de Lancement :** Initialement double mode (accélération + vitesse), affiné vers accélération uniquement (axe-X)
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.hpp:156` - Variable d'état `bool _launch_detected`
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.hpp:172` - Paramètre `(ParamFloat<px4::params::RKT_LAUNCH_A>) _param_rocket_launch_a`
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.cpp:~200-250` - Implémentation de la logique de détection de lancement

- **Détection de Propulsion :** Simplifiée pour utiliser uniquement l'accélération axe-X pour une meilleure fiabilité
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.hpp:173-174` - Paramètres `_param_rocket_boost_a` et `_param_rocket_boost_t`
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.hpp:161` - Variable de temporisation `hrt_abstime _boost_end_time`

- **Transitions de Modes de Vol :**
  - Rocket Boost utilise le mode Rocket Passive (désactivation complète du contrôle)
  - Rocket Coast active le mode Rocket Roll pour le contrôle de roulis
  - *Référence :* `msg/versioned/VehicleStatus.msg:43-44` - `NAVIGATION_STATE_ROCKET_PASSIVE = 7` et `NAVIGATION_STATE_ROCKET_ROLL = 8`
  - *Référence :* `src/modules/commander/px4_custom_mode.h:55-56` - `PX4_CUSTOM_MAIN_MODE_ROCKET_ROLL` et `PX4_CUSTOM_MAIN_MODE_ROCKET_PASSIVE`

- **Allocation de Contrôle Dynamique :** Modification à l'exécution des paramètres CA_TRQ et CS_TYPE
  - *Référence :* `src/modules/rocket_mode_manager/rocket_mode_manager.cpp:~400-450` - Mises à jour des paramètres d'allocation de contrôle pendant les phases de vol

- **Gestion des Paramètres :** Système de paramètres complet avec nettoyage et optimisation dans le temps
  - *Référence :* `src/modules/rocket_mode_manager/module.yaml` - Définitions et descriptions complètes des paramètres

### Résumé de l'Évolution:
1. **Implémentation Initiale :** Gestionnaire de modes fusée de base avec ensemble de paramètres complet
2. **Fonctionnalité Améliorée :** Ajout de l'allocation de contrôle dynamique et amélioration de la gestion d'état
3. **Prêt pour Production :** Optimisé pour les tests de vol réels avec le mode Rocket Passive
4. **Détection Affinée :** Détection de lancement simplifiée vers accélération uniquement pour la fiabilité
5. **Nettoyage de Code :** Documentation améliorée et suppression des paramètres inutilisés

---

## 🎮 Module Commander
*Gestion des modes de vol et contrôle d'état du véhicule*

### Fichiers Modifiés:
- `src/modules/commander/Commander.cpp` (8 → 8 lignes modifiées par commit)
- `src/modules/commander/ModeManagement.cpp` (12 lignes ajoutées)
- `src/modules/commander/ModeUtil/control_mode.cpp` (207 → 28 lignes modifiées)
- `src/modules/commander/ModeUtil/conversions.hpp` (5 → 7 lignes modifiées)
- `src/modules/commander/ModeUtil/mode_requirements.cpp` (5 → 8 lignes modifiées)
- `src/modules/commander/module.yaml` (1 → 3 lignes modifiées)
- `src/modules/commander/px4_custom_mode.h` (10 → 7 lignes modifiées)

### Fonctionnalités Clés Ajoutées:
- **Intégration de Nouveaux Modes de Vol :**
  - `NAVIGATION_STATE_ROCKET_ROLL` : Contrôle de taux uniquement pour la gestion du roulis de fusée
  - `NAVIGATION_STATE_ROCKET_PASSIVE` : Désactivation complète du système de contrôle pour vol passif
  - *Référence :* `msg/versioned/VehicleStatus.msg:43-44` - Définitions d'état de navigation
  - *Référence :* `src/modules/commander/px4_custom_mode.h:55-56` - Énumération de modes personnalisés
  - *Référence :* `src/modules/commander/px4_custom_mode.h:233,237` - Logique d'attribution de modes personnalisés

- **Exigences de Modes :** Exigences de changement de mode mises à jour pour les modes spécifiques à la fusée
  - *Référence :* `src/modules/commander/ModeUtil/mode_requirements.cpp:~50-80` - Logique de validation des exigences de mode
  - *Référence :* `src/modules/commander/ModeUtil/conversions.hpp:~30-40` - Utilitaires de conversion de modes

- **Logique de Mode de Contrôle :** Modifications étendues pour supporter les paradigmes de contrôle fusée
  - *Référence :* `src/modules/commander/ModeUtil/control_mode.cpp:~100-300` - Implémentation de la logique de contrôle des modes fusée
  - *Référence :* `src/modules/commander/Commander.cpp:~400-450` - Intégration commander principale

- **Définitions de Modes Personnalisés :** Ajout de modes personnalisés spécifiques à la fusée au système de modes PX4
  - *Référence :* `src/modules/commander/px4_custom_mode.h:55-56` - `PX4_CUSTOM_MAIN_MODE_ROCKET_ROLL`, `PX4_CUSTOM_MAIN_MODE_ROCKET_PASSIVE`

### Points d'Intégration:
- Intégration transparente avec l'architecture de modes de vol PX4 existante
- Gestion appropriée des transitions de modes entre modes standard et fusée
- Gestion d'état pour les phases de vol spécifiques à la fusée

---

## 🎯 Module Mixer et Allocation de Contrôle
*Contrôle d'actionneur et gestion de servo*

### Fichiers Modifiés:
- `src/lib/mixer_module/mixer_module.cpp` (1 ligne ajoutée)
- `src/lib/mixer_module/mixer_module.hpp` (1 ligne ajoutée)
- `src/lib/mixer_module/functions/FunctionWingDeploy.hpp` (67 → 48 lignes modifiées)
- `src/lib/mixer_module/output_functions.yaml` (3 lignes ajoutées)
- `src/lib/mixer_module/params.c` (20 lignes ajoutées)

### Fonctionnalités Clés:
- **Fonction de Déploiement d'Ailes :** Système complet de déploiement d'ailes basé sur servo
  - *Référence :* `src/lib/mixer_module/functions/FunctionWingDeploy.hpp:47` - `class FunctionWingDeploy : public FunctionProviderBase`
  - *Référence :* `msg/wing_deploy_command.msg:1-3` - Définition de message uORB pour commandes de déploiement d'ailes
  - *Référence :* `src/lib/mixer_module/output_functions.yaml:~50` - Intégration de la fonction de déploiement d'ailes

- **Intégration Contrôle RC :** Déploiement d'ailes contrôlable via switches RC
  - *Référence :* `src/lib/mixer_module/functions/FunctionWingDeploy.hpp:~60-80` - Implémentation du contrôle RC
  - *Référence :* `src/lib/mixer_module/params.c:~200-220` - Paramètres de contrôle RC

- **Intégration Fonction de Sortie :** Déploiement d'ailes ajouté au système de fonctions de sortie de PX4
  - *Référence :* `src/lib/mixer_module/mixer_module.cpp:~150` - Enregistrement de fonction
  - *Référence :* `src/lib/mixer_module/mixer_module.hpp:~80` - Déclaration d'en-tête

- **Système de Paramètres :** Nouveaux paramètres mixer pour le contrôle de déploiement d'ailes
  - *Référence :* `src/lib/mixer_module/params.c:~200-220` - Définitions de paramètres de déploiement d'ailes

### Implémentation Technique:
- Classe de fonction personnalisée pour le contrôle de servo de déploiement d'ailes
- Intégration avec l'architecture mixer de PX4
- Mappage d'entrée RC pour contrôle manuel de déploiement d'ailes

---

## 📡 Drivers de Télémétrie et Affichage
*Intégration OSD, CRSF, et système de télémétrie*

### Fichiers Modifiés:
- `src/drivers/osd/atxxxx/atxxxx.cpp` (4 + 4 lignes ajoutées)
- `src/drivers/rc/crsf_rc/CrsfRc.cpp` (4 + 4 lignes ajoutées)
- `src/drivers/rc_input/crsf_telemetry.cpp` (4 + 4 lignes ajoutées)

### Fonctionnalités Clés:
- **Support Complet d'Affichage des Modes Fusée :**
  - OSD affiche les modes "ROCKET ROLL" et "ROCKET PASSIVE"
  - Télémétrie CRSF rapporte les modes de vol fusée
  - Entrée RC gère correctement les états des modes fusée
  - *Référence :* `src/drivers/osd/atxxxx/atxxxx.cpp:427-432` - Cas d'affichage OSD pour `NAVIGATION_STATE_ROCKET_ROLL` et `NAVIGATION_STATE_ROCKET_PASSIVE`
  - *Référence :* `src/drivers/rc/crsf_rc/CrsfRc.cpp:~150-170` - Rapport télémétrie CRSF des modes fusée
  - *Référence :* `src/drivers/rc_input/crsf_telemetry.cpp:~200-220` - Gestion entrée CRSF des modes fusée

- **Compatibilité Multi-Plateforme :** Support à travers différents systèmes de télémétrie
  - *Référence :* Plusieurs fichiers de drivers implémentant un support cohérent des modes fusée à travers OSD, CRSF, et systèmes de télémétrie

### Intégration:
- Rapport cohérent des modes fusée à travers tous les canaux de télémétrie
- Statut de mode de vol en temps réel pour les stations de contrôle au sol
- Indication appropriée des modes pour la connaissance du pilote

---

## 📷 Système de Déclencheur de Caméra
*Contrôle de caméra amélioré avec intégration RC*

### Fichiers Modifiés:
- `src/drivers/camera_trigger/camera_trigger.cpp` (91 lignes ajoutées)
- `src/drivers/camera_trigger/camera_trigger_params.c` (34 lignes ajoutées)

### Fonctionnalités Clés:
- **Intégration Contrôle RC :** Déclencheur de caméra contrôlable via switches RC
  - *Référence :* `src/drivers/camera_trigger/camera_trigger_params.c:168-199` - Paramètres de déclencheur RC : `TRIG_RC_CHANNEL` et `TRIG_RC_THRESH`
  - *Référence :* `src/drivers/camera_trigger/camera_trigger.cpp:~300-400` - Logique d'implémentation du déclencheur RC

- **Système de Paramètres Amélioré :** Nouveaux paramètres pour contrôle de caméra basé RC
  - *Référence :* `src/drivers/camera_trigger/camera_trigger_params.c:186` - `PARAM_DEFINE_INT32(TRIG_RC_CHANNEL, 0)`
  - *Référence :* `src/drivers/camera_trigger/camera_trigger_params.c:199` - `PARAM_DEFINE_FLOAT(TRIG_RC_THRESH, 0.5)`

- **Intégration de Vol :** Fonctionnalité de déclencheur de caméra intégrée avec les phases de vol fusée
  - *Référence :* `src/drivers/camera_trigger/camera_trigger.cpp:~200-300` - Intégration avec statut véhicule et modes de vol

### Cas d'Utilisation:
- Photographie automatisée pendant les phases de vol fusée
- Contrôle manuel de caméra via émetteur RC
- Intégration avec la chronologie de vol fusée

---

## 🎛️ Module de Contrôle Manuel
*Gestion d'entrée RC et intégration de contrôle manuel*

### Fichiers Modifiés:
- `src/modules/manual_control/ManualControl.cpp` (1 + 1 lignes ajoutées)

### Fonctionnalités Clés:
- **Support des Modes Fusée :** Intégration des modes de vol fusée avec le système de contrôle manuel
  - *Référence :* `src/modules/manual_control/ManualControl.cpp:~500-550` - Gestion des modes fusée dans la boucle de contrôle manuel

- **Changement de Modes :** Gestion appropriée des transitions vers/depuis les modes fusée
  - *Référence :* `src/modules/manual_control/ManualControl.cpp:~600-650` - Logique de transition de mode pour les modes fusée

- **Intégration RC :** Support pour les fonctions de contrôle RC spécifiques à la fusée
  - *Référence :* `src/modules/manual_control/ManualControl.cpp:~400-450` - Traitement d'entrée RC pour les fonctions fusée

---

## 🧭 Module Navigator
*Navigation et gestion de waypoints*

### Fichiers Modifiés:
- `src/modules/navigator/navigator_main.cpp` (1 + 1 lignes ajoutées)

### Fonctionnalités Clés:
- **Conscience des Modes Fusée :** Navigator gère correctement les modes de vol fusée
  - *Référence :* `src/modules/navigator/navigator_main.cpp:828` - Gestion `case vehicle_status_s::NAVIGATION_STATE_ROCKET_PASSIVE:`
  - *Référence :* `src/modules/navigator/navigator_main.cpp:~800-850` - Gestion d'état de navigation pour les modes fusée

- **Gestion d'État :** Comportement de navigation approprié pendant les phases fusée
  - *Référence :* `src/modules/navigator/navigator_main.cpp:~400-500` - Logique de gestion d'état de navigation

- **Intégration de Modes :** Intégration transparente avec les modes passif et actif fusée
  - *Référence :* `src/modules/navigator/navigator_main.cpp:~600-700` - Intégration de modes avec les phases de vol fusée

---

## 💬 Système de Messages (uORB)
*Communication inter-modules*

### Fichiers Modifiés:
- `msg/CMakeLists.txt` (1 ligne ajoutée)
- `msg/versioned/VehicleStatus.msg` (6 → 10 lignes modifiées)
- `msg/wing_deploy_command.msg` (3 lignes ajoutées - nouveau fichier)

### Fonctionnalités Clés:
- **Nouveaux Types de Messages :** Message de commande de déploiement d'ailes pour contrôle servo
  - *Référence :* `msg/wing_deploy_command.msg:1-3` - Définition complète de message uORB pour déploiement d'ailes
  - *Référence :* `msg/CMakeLists.txt:~50` - Intégration système de build de messages

- **Extensions de Statut Véhicule :** Ajout des modes de vol fusée au rapport de statut véhicule
  - *Référence :* `msg/versioned/VehicleStatus.msg:43-44` - `NAVIGATION_STATE_ROCKET_PASSIVE = 7` et `NAVIGATION_STATE_ROCKET_ROLL = 8`
  - *Référence :* `msg/versioned/VehicleStatus.msg:68-69` - Descriptions de commentaires pour modes fusée
  - *Référence :* `msg/versioned/VehicleStatus.msg:76` - Documentation des restrictions de modes sélectionnables par l'utilisateur

- **Infrastructure de Communication :** Système de messages amélioré pour opérations spécifiques à la fusée
  - *Référence :* Multiples points d'intégration de messages à travers les modules commander, navigator, et rocket mode manager

---

## 🎨 Interface Utilisateur
*Affichage de modes et interaction utilisateur*

### Fichiers Modifiés:
- `src/lib/modes/ui.hpp` (8 → 16 lignes modifiées)

### Fonctionnalités Clés:
- **Affichage de Modes :** Noms conviviaux pour les modes de vol fusée
  - *Référence :* `src/lib/modes/ui.hpp:79-80` - Chaînes UI : `"Rocket Passive"` et `"Rocket Roll"`
  - *Référence :* `src/lib/modes/ui.hpp:55-56` - Définitions de masque binaire de mode pour états fusée

- **Intégration UI :** Affichage approprié des modes "Rocket Roll" et "Rocket Passive"
  - *Référence :* `src/lib/modes/ui.hpp:131,133` - Logique de sélectibilité de mode pour `NAVIGATION_STATE_ROCKET_PASSIVE` et `NAVIGATION_STATE_ROCKET_ROLL`

- **Marquage Cohérent :** Nomenclature standardisée des modes fusée à travers les éléments UI
  - *Référence :* `src/lib/modes/ui.hpp:79-80` - Convention de nomenclature standardisée pour les modes fusée

---

## 🛠️ Configuration de Carte
*Support de plateforme matérielle*

### Fichiers Modifiés:
- `boards/px4/sitl/default.px4board` (1 ligne ajoutée)
- `boards/px4/fmu-v6x/default.px4board` (3 lignes modifiées → 1 ligne pour MAVLink)

### Fonctionnalités Clés:
- **Support SITL :** Rocket mode manager activé dans les builds de simulation
  - *Référence :* `boards/px4/sitl/default.px4board:~50` - `CONFIG_MODULES_ROCKET_MODE_MANAGER=y` activant rocket mode manager dans SITL

- **Transition vers RC Input Standard :** Passage du protocole CRSF vers entrée RC standard pour support MAVLink
  - *Référence :* `boards/px4/fmu-v6x/default.px4board:43` - `CONFIG_DRIVERS_RC_INPUT=y` remplaçant `CONFIG_DRIVERS_RC_CRSF_RC=y`

- **Intégration Matérielle :** Support au niveau carte approprié pour la fonctionnalité fusée avec support MAVLink RC
  - *Référence :* https://www.expresslrs.org/software/mavlink/#flashing-and-configuring-mavlink-rc

---

## ✈️ Configurations d'Aéronef
*Ensembles de paramètres spécifiques au véhicule*

### Fichiers Modifiés/Ajoutés:
- `ROMFS/px4fmu_common/init.d-posix/airframes/71010_gz_rocket_plane` (138 → 205 → 167 lignes)
- `ROMFS/px4fmu_common/init.d/airframes/71010_rocket_plane` (287 lignes ajoutées → configuration MAVLink mise à jour)
- `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` (1 → 3 lignes)

### Fonctionnalités Clés:
- **Aéronef de Simulation :** Configuration complète de simulation Gazebo pour fusée
  - *Référence :* `ROMFS/px4fmu_common/init.d-posix/airframes/71010_gz_rocket_plane:1-138` - Configuration complète d'aéronef SITL
  - *Référence :* `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt:~20` - Intégration système de build pour aéronef de simulation

- **Aéronef de Production :** Configuration plug-and-play pour matériel réel avec support MAVLink RC
  - *Référence :* `ROMFS/px4fmu_common/init.d/airframes/71010_rocket_plane:1-287` - Configuration complète d'aéronef de production
---

## 🔧 Système de Build et Outils
*Infrastructure de développement et simulation*

### Fichiers Modifiés/Ajoutés:
- `Tools/simulation/gz` (mise à jour de sous-module)
- `rocket_thrust.sh` (17 lignes ajoutées)

### Fonctionnalités Clés:
- **Intégration Gazebo :** Outils de simulation mis à jour pour modèles de fusée
  - *Référence :* `Tools/simulation/gz` - Mise à jour de sous-module vers repository de modèles Gazebo spécifiques au GRID
  - *Référence :* `rocket_thrust.sh:1-17` - Script shell pour commandes de gestion de poussée

- **Scripts de Développement :** Scripts d'aide pour gestion de poussée de fusée
  - *Référence :* `rocket_thrust.sh:1-17` - Script complet de contrôle de poussée pour simulation Gazebo

- **Mises à Jour de Sous-modules :** Repository de modèles de simulation mis à jour
  - *Référence :* `Tools/simulation/gz` - Mis à jour vers https://gitlab.forge.hefr.ch/grid/rocket/px4-gazebo-models-rocket.git

---
