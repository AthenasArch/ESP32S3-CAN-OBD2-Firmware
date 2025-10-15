/**
 * @file sensorsTask.cpp
 * @brief Exemplo de controle de menus via terminal/Serial utilizando a biblioteca AthenasButton.
 *
 * @details
 *  - Demonstra o uso de 4 botões (Left, Right, Enter e OnBoard) para navegar por menus
 *    e executar ações simples, validando as funcionalidades: CLICK, PRESS_MEDIUM, auto-repeat
 *    e (opcionalmente) múltiplos cliques quando habilitado na biblioteca.
 *  - Padrão de uso:
 *      1) Configuração dos botões com begin() / beginAdvanced();
 *      2) Chamar loop() para cada botão em alta periodicidade (aqui, a cada 10 ms);
 *      3) Consumir eventos com available() + read() centralizados nas rotinas handle*;
 *      4) Centralizar a impressão (printMenuUpdate) para evitar logs duplicados.
 *
 *  - Regras de navegação adotadas:
 *      * Fora de submenu:
 *          - Right/Left: percorrem os itens de menu com auto-repeat (wrap-around).
 *          - Enter: alterna entre entrar/sair do submenu atual (via CLICK).
 *          - OnBoard:
 *              - CLICK        : avança 1 menu (wrap-around).
 *              - PRESS_MEDIUM : entra diretamente no submenu "Contador".
 *      * Dentro do submenu:
 *          - Right/Left: ajustam variáveis (ex.: contador).
 *          - OnBoard:
 *              - PRESS_MEDIUM no "Contador": sai do submenu (retorna ao menu).
 *
 *  - Integração:
 *      * Adequado para execução em tarefa RTOS (FreeRTOS) — ver sensors_Task().
 */

#include "taskManager/Tasks/sensorsTask.hpp"
#include "taskManager/taskManager.h"
#include "modules/ADC/adArch.h"
#include "Hardware/hardware.h"
#include "AthenasButton.hpp"

/**
 * @enum MenuState
 * @brief Estados possíveis do menu de demonstração.
 *
 * @details
 *  - MENU_MAIN     : Tela/estado principal.
 *  - MENU_COUNTER  : Submenu que ajusta um contador (0..999).
 *  - MENU_SETTINGS : Espaço para configurações (exemplo).
 *  - MENU_INFO     : Espaço para informações (exemplo).
 *  - MENU_TOTAL    : Sentinela para contagem (não é um estado navegável).
 */
enum MenuState {
    MENU_MAIN = 0,
    MENU_COUNTER,
    MENU_SETTINGS,
    MENU_INFO,
    MENU_TOTAL
};

/** @brief Botão de navegação para esquerda (decremento). */
AthenasButton btnLeft (PIN_PCI_ATHENAS_SW_LEFT);
/** @brief Botão de navegação para direita (incremento). */
AthenasButton btnRight(PIN_PCI_ATHENAS_SW_RIGHT);
/** @brief Botão ENTER (toggle entrar/sair do submenu). */
AthenasButton btnEnter(PIN_PCI_ATHENAS_SW_ENTER);
/** @brief Botão OnBoard (boot) com regras especiais de entrada rápida. */
AthenasButton btnOnBoard(PIN_BNT_BOOT);

// ---------------------------- Estado Global do Demo ----------------------------

/** @brief Estado atual do menu. */
static MenuState currentMenu = MENU_MAIN;
/** @brief Indica se estamos dentro de um submenu (true) ou na lista de menus (false). */
static bool insideMenu = false;
/** @brief Variável usada no submenu "Contador". */
static int counterValue = 0;

/** @brief Último contador impresso (para evitar prints duplicados). */
static int        lastCounterValue = -999;
/** @brief Último menu impresso (para evitar prints duplicados). */
static MenuState  lastMenuPrinted  = MENU_TOTAL;
/** @brief Flag para detectar transição entrar/sair de submenu. */
static bool       lastInsideMenu   = false;

/** @brief Nomes legíveis dos menus (índice em sincronia com MenuState). */
const char* menuNames[] = {
    "Principal",
    "Contador",
    "Configurações",
    "Informações"
};

// ---------------------------- Protótipos internos ----------------------------
/** @brief Inicializa a task: Serial e configuração dos botões. */
static void menuButton_ini(SystemStatus *systemStatus);
/** @brief Loop principal da task: atualiza botões, processa eventos e imprime logs. */
static void menuButton_run(SystemStatus *systemStatus);
/** @brief Trata o botão ENTER: alterna entrar/sair do submenu via CLICK. */
static void handleMenuEnter();
/** @brief Trata navegação de menus com Right/Left quando NÃO está em submenu. */
static void handleMenuNavigation();
/** @brief Executa ações do submenu selecionado (ex.: contador). */
static void handleMenuExecution();
/** @brief Regras especiais do botão OnBoard (entrada rápida, sair do contador). */
static void handleOnBoardButton();
/** @brief Centraliza as impressões para evitar duplicidade de logs. */
static void printMenuUpdate();

// ---------------------------- Task FreeRTOS de Exemplo ----------------------------

/**
 * @brief Task principal de sensores (exemplo). Aqui apenas orquestra a UI de botões.
 * @param pvParameters Ponteiro para SystemStatus (não utilizado no exemplo de UI).
 *
 * @details
 *  - Inicializa a UI de botões;
 *  - Executa o laço infinito chamando menuButton_run() a cada 10 ms;
 *  - A função vTaskDelay cede a CPU no RTOS.
 */
void sensors_Task(void* pvParameters) {
    SystemStatus *systemStatus = (SystemStatus *)pvParameters;

    menuButton_ini(systemStatus);

    for (;;) {
        menuButton_run(systemStatus);
        vTaskDelay(pdMS_TO_TICKS(10)); // ~10 ms → bom compromisso entre latência e CPU
    }
}

// ---------------------------- Inicialização ----------------------------

/**
 * @brief Configura a Serial e inicializa os 4 botões com seus perfis por PARÂMETROS.
 * @param systemStatus Ponteiro para estado do sistema (não usado aqui).
 *
 * @details
 *  - Left/Right: navegação com auto-repeat em 3 estágios; debounce/interval ajustados.
 *  - Enter: modo simples, apenas CLICK, para entrar/sair do submenu (sem duplo clique).
 *  - OnBoard: hold médio aos 3000 ms, long aos 5000 ms, "fire-on-hold" habilitado,
 *             suprime CLICK no release após hold-event e permite upgrade medium→long.
 *
 * @note Sobre múltiplos cliques:
 *  - Por padrão, multi-clique está desligado → sem atrasos e sem DOUBLE/TRIPLE.
 *  - Se quiser duplo clique sem atrasar o single:
 *        setSimpleMode(false);
 *        setClickInterval(300);
 *        setMultiClick(true, false)  // deferSingle=false
 *  - Se quiser duplo clique com atraso do single (single “limpo”):
 *        setSimpleMode(false);
 *        setClickInterval(300);
 *        setMultiClick(true, true)   // deferSingle=true
 */
static void menuButton_ini(SystemStatus *systemStatus) {
    Serial.begin(115200);

    // LEFT e RIGHT: navegação com repeat (1 chamada avançada por botão)
    btnLeft.beginAdvanced( /*medium*/300, /*long*/1000,
                           /*fireOnHold*/false,
                           /*lockReleaseAfterHold*/true,
                           /*allowLongUpgrade*/false,
                           /*autoRepeat*/true,
                           /*debounce*/60,
                           /*clickInterval*/300,
                           /*simple*/true,
                           /*usePullup*/true,
                           /*activeLow*/true,
                           /*ar1Delay*/600,  /*ar1Int*/600,
                           /*ar2Delay*/4000, /*ar2Int*/200,
                           /*ar3Int*/100 );

    btnRight.beginAdvanced(300, 1000, false, true, false, true,
                           60, 300, true, true, true,
                           600, 600, 4000, 200, 100);

    // ENTER: toggle via clique (versão curta, sem ambiguidade)
    btnEnter.begin( 400, 1500, /*fireOnHold*/false, /*autoRepeat*/false );
    // Explicitamente, se quiser reforçar:
    // btnEnter.setSimpleMode(true);
    // btnEnter.setMultiClick(false);

    // ONBOARD: MEDIUM=3000ms, LONG=5000ms, fire-on-hold, lock release, upgrade
    btnOnBoard.beginAdvanced( 3000, 5000,
                              /*fireOnHold*/true,
                              /*lockReleaseAfterHold*/true,
                              /*allowLongUpgrade*/true,
                              /*autoRepeat*/false,
                              /*debounce*/60,
                              /*clickInterval*/350,
                              /*simple*/true,
                              /*usePullup*/true,
                              /*activeLow*/true,
                              /*ar1Delay*/1000, /*ar1Int*/1000,
                              /*ar2Delay*/6000, /*ar2Int*/250,
                              /*ar3Int*/100 );
}

// ---------------------------- Loop da Task ----------------------------

/**
 * @brief Atualiza todos os botões, processa eventos e imprime mudanças.
 * @param systemStatus Ponteiro para estado do sistema (não usado aqui).
 *
 * @details
 *  - Chama loop() dos 4 botões (atualiza máquina de estados interna);
 *  - Processa eventos de ENTER e OnBoard;
 *  - Direciona para navegação (fora de submenu) ou execução (dentro do submenu);
 *  - Centraliza logs em printMenuUpdate().
 */
static void menuButton_run(SystemStatus *systemStatus) {
    // Atualiza estado dos botões (debounce, transições, eventos)
    btnLeft.loop();
    btnRight.loop();
    btnEnter.loop();
    btnOnBoard.loop();

    // Processamento central de eventos
    handleMenuEnter();
    handleOnBoardButton();

    // Roteia para navegação ou execução conforme insideMenu
    if (!insideMenu) handleMenuNavigation();
    else             handleMenuExecution();

    // Impressões centralizadas para evitar duplicidades
    printMenuUpdate();
}

// ---------------------------- Tratamento do ENTER ----------------------------

/**
 * @brief Alterna entrar/sair do submenu via CLICK no botão ENTER.
 *
 * @details
 *  - Consome todos os eventos pendentes do ENTER;
 *  - Ignora outros tipos de eventos para manter a UX simples.
 */
static void handleMenuEnter() {
    while (btnEnter.available()) {
        const AthenasButtonEvent ev = btnEnter.read();
        if (ev == BTN_CLICK) {
            insideMenu = !insideMenu; // toggle
        }
    }
}

// ---------------------------- Tratamento do OnBoard ----------------------------

/**
 * @brief Regras do botão OnBoard (boot).
 *
 * @details
 *  - Fora do submenu:
 *      - CLICK         : avança 1 posição no menu (wrap-around).
 *      - PRESS_MEDIUM  : entra direto no submenu "Contador".
 *  - Dentro do submenu:
 *      - Se estiver no "Contador" e ocorrer PRESS_MEDIUM: sai do submenu.
 *
 * @note Consome todos os eventos pendentes do OnBoard.
 */
static void handleOnBoardButton() {
    while (btnOnBoard.available()) {
        const AthenasButtonEvent ev = btnOnBoard.read();

        if (!insideMenu) {
            if (ev == BTN_CLICK) {
                // Avança o menu 1 passo (sem casts; enum contíguo)
                btnOnBoard.bumpEnumByCount(currentMenu, MENU_MAIN, MENU_TOTAL, +1);
            } else if (ev == BTN_PRESS_MEDIUM) {
                currentMenu = MENU_COUNTER;
                insideMenu = true;
            }
        } else {
            if (currentMenu == MENU_COUNTER && ev == BTN_PRESS_MEDIUM) {
                insideMenu = false;
            }
        }
    }
}

// ---------------------------- Navegação de Menus (fora do submenu) ----------------------------

/**
 * @brief Usa Right (this) e Left (btnLeft) para navegar na lista de menus.
 *
 * @details
 *  - processMenuByCountDrainAll: drena todos os eventos pendentes de ambos os botões;
 *  - Navegação com wrap-around entre MENU_MAIN .. MENU_TOTAL-1.
 */
static void handleMenuNavigation() {
    btnRight.processMenuByCountDrainAll(btnLeft, currentMenu, MENU_MAIN, MENU_TOTAL);
}

// ---------------------------- Execução dos Menus (dentro do submenu) ----------------------------

/**
 * @brief Executa ações do submenu selecionado.
 *
 * @details
 *  - MENU_COUNTER: Right/Left ajustam counterValue (0..999) com wrap-around.
 *  - MENU_SETTINGS / MENU_INFO: exemplos simples que reagem a CLICK do Right.
 */
static void handleMenuExecution() {
    switch (currentMenu) {
        case MENU_COUNTER:
            // Ajuste com wrap: Right incrementa / Left decrementa
            btnRight.processMenuDrainAll(btnLeft, counterValue, 0, 999);
            break;

        case MENU_SETTINGS:
            while (btnRight.available()) {
                if (btnRight.read() == BTN_CLICK) {
                    /* ação de configuração (exemplo) */
                }
            }
            break;

        case MENU_INFO:
            while (btnRight.available()) {
                if (btnRight.read() == BTN_CLICK) {
                    /* ação de informação (exemplo) */
                }
            }
            break;

        default:
            break;
    }
}

// ---------------------------- Impressão Controlada ----------------------------

/**
 * @brief Centraliza logs de estado para evitar duplicação de mensagens no Serial.
 *
 * @details
 *  - Loga transições entrar/sair do submenu;
 *  - Loga mudança de menu somente quando NÃO está em submenu;
 *  - Loga mudança do contador apenas quando estiver no submenu "Contador".
 */
static void printMenuUpdate() {
    // 1) Entrou/Saiu do submenu
    if (insideMenu != lastInsideMenu) {
        if (insideMenu) {
            Serial.printf(">>> Entrou no submenu: %s\n", menuNames[currentMenu]);
        } else {
            Serial.printf("<<< Saiu para menu: %s\n", menuNames[currentMenu]);
        }
        lastInsideMenu = insideMenu;
    }

    // 2) Mudança de menu (somente fora de submenu)
    if (!insideMenu && currentMenu != lastMenuPrinted) {
        Serial.printf(">>> Mudou para menu: %s\n", menuNames[currentMenu]);
        lastMenuPrinted = currentMenu;
    }

    // 3) Atualização do contador (somente dentro do submenu "Contador")
    if (insideMenu && currentMenu == MENU_COUNTER && counterValue != lastCounterValue) {
        Serial.printf("Counter atualizado: %d\n", counterValue);
        lastCounterValue = counterValue;
    }
}
