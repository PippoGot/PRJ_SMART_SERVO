# Specifiche per la classe da implementare:

- Valori di inizializzazione:
    + vcc  ->  Tensione massima applicabile (valore corrispondente a un duty-cycle = 100%);

    + (Da implementare più avanti) Piedini corrispondenti per le uscite, 2x PWM, 2x Digitali;


- Metodi:
    + apply_PWM  ->  ha come ingresso un numero compreso tra 0 (0%) e 1 (100%) corrispondenti al
    duty-cycle da imporre in uscita.
    Il modulo determina il duty-cycle, il segno determina la direzione.

    A = uscita digitale per direzione 1
    B = uscita PWM per direzione 2
    C = uscita digitale per direzione 2
    D = uscita PWM per direzione 1

    + apply_voltage  ->  ha come ingresso una tensione che non può essere maggiore di "vcc"
    in modulo, ma può invece essere positiva o negativa.
    Calcola il rapporto tra la tensione voluta (l'ingresso) e la tensione massima applicabile
    ("vcc"), successivamente applica il PWM ai piedini di uscita come specificato dalla funzione
    apposita (utilizza quella funzione);